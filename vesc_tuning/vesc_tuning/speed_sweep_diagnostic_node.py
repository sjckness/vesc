#!/usr/bin/env python3
"""Commanded-vs-actual speed sweep -- a read-only diagnostic to characterize
mechanical losses (motor/drivetrain slip, static friction, etc.) BEFORE
running sensor_covariance_calibration_node's light_motion vx_variance
calibration. Does not touch sensor_covariance_calibration_node, vesc.yaml, or
any other config file -- this only ever prints to console and, optionally,
writes a standalone results CSV.

Ground truth here is deliberately manual: no onboard sensor (wheel odom, IMU,
ZED, ...) is trusted for "actual speed", since characterizing exactly how far
those sensors' assumptions (e.g. a fixed speed_to_erpm_gain) diverge from
reality is the whole point. actual_speed is always measured_distance / duration
from a human with a tape measure -- never a topic.

Human-in-the-loop pattern (mirrors this package's speed_tuning_node.py /
steering_calibration_node.py, and sensor_covariance_calibration_node.py's
light_motion mode): a stop command is published immediately on startup, every
drive is gated by a blocking input() confirmation naming the exact upcoming
command, and a stop is always published again afterward and in every
exit path (normal completion, 'q' abort, Ctrl-C, or exception).

Unlike speed_tuning_node.py / steering_calibration_node.py, this node has no
subscriptions or timers -- there is nothing for an executor to dispatch, so
(unlike those two, which spin on a background thread for structural
consistency with nodes that DO need it) this one just runs its interactive
loop directly; rclpy publishers do not require spinning to actually publish.

For each target speed in `target_speeds` (m/s):
  1. Blocking confirmation naming the exact target speed and duration.
  2. Drive at that constant commanded speed for `drive_duration_sec`, then stop.
  3. Prompt for the measured distance traveled (meters, human tape measure).
  4. Print target vs actual vs absolute/percent error immediately.
After the sweep (whether completed or aborted early with data already
collected), print a summary table sorted by target speed, and optionally
write it to a CSV file under this package's speed_sweep_results/ directory.
"""

import csv
import datetime
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

PUBLISH_RATE_HZ = 50.0
SEP = '=' * 78


def _default_results_dir() -> Path:
    """Resolve <vesc_tuning package root>/speed_sweep_results, reliably
    reaching the real source tree regardless of whether this workspace was
    built with --symlink-install.

    Same technique as sensor_covariance_calibration_node.resolve_source_vesc_
    yaml_path() (see that function's docstring for the full reasoning) --
    reimplemented standalone here rather than imported, since the two nodes
    live in unrelated packages and this one must not depend on
    f1tenth_diagnostics. Verified on this workspace: without
    --symlink-install, ament_python installs this file itself as a plain
    copy under install/vesc_tuning/lib/python3.10/site-packages/..., so a
    naive parents[N] anchor off __file__ alone is NOT reliable -- walking up
    for an 'install' ancestor (a colcon workspace-layout invariant regardless
    of --symlink-install) is.
    """
    this_file = Path(__file__).resolve()
    for parent in this_file.parents:
        if parent.name == 'install':
            ws_root = parent.parent
            return (ws_root / 'src' / 'f1tenth_hardware' / 'vesc' / 'vesc_tuning'
                     / 'speed_sweep_results')

    # No 'install' ancestor -- this file's own resolved location IS inside
    # src/ already (editable install via --symlink-install).
    # parents[0] = vesc_tuning/vesc_tuning (this file's dir)
    # parents[1] = vesc_tuning (the package root, containing setup.py)
    return this_file.parents[1] / 'speed_sweep_results'


class SpeedSweepDiagnosticNode(Node):

    def __init__(self):
        super().__init__('speed_sweep_diagnostic_node')

        # Densified (velocity-correction pass): the original 6-point sweep
        # (0.15-0.4) already found serious nonlinearity in this exact band
        # (0.15->0.083 m/s, 44% error, the worst point measured) -- extended
        # down to 0.05 (below the previous floor, where deadband/static
        # friction is expected to matter most and was previously
        # uncharacterized) and up through 0.50 (the one point known-good via
        # speed_tuning_node's own separate calibration, kept here too so the
        # correction curve's top knot lines up with where the existing
        # linear speed_to_erpm_gain is already known to track well -- see
        # velocity_correction.py's own module docstring for how that boundary
        # is used). Same human-in-the-loop measurement method as before,
        # unchanged -- only the target set is denser.
        target_speeds = self.declare_parameter(
            'target_speeds',
            [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]).value
        self.target_speeds = [float(v) for v in target_speeds]
        self.drive_duration_sec = float(
            self.declare_parameter('drive_duration_sec', 3.0).value)

        # Plain node-level defaults matching vesc.yaml's current values --
        # same convention sensor_covariance_calibration_node.py's light_motion
        # mode uses for the same purpose (known vehicle constants needed to
        # convert a target m/s into an erpm command, not calibration-workflow
        # knobs, so not read from vesc.yaml -- this node never touches it).
        self.speed_to_erpm_gain = float(
            self.declare_parameter('speed_to_erpm_gain', 5499.271647286143).value)
        self.speed_to_erpm_offset = float(
            self.declare_parameter('speed_to_erpm_offset', 0.0).value)
        self.steering_center = float(
            self.declare_parameter('steering_center', 0.5304).value)
        self.command_topic = str(
            self.declare_parameter('command_topic', '/commands/motor/speed').value)
        self.servo_topic = str(
            self.declare_parameter('servo_topic', '/commands/servo/position').value)

        self.write_csv = bool(self.declare_parameter('write_csv', True).value)
        self.csv_path = str(self.declare_parameter('csv_path', '').value)

        self._speed_pub = self.create_publisher(Float64, self.command_topic, 10)
        self._servo_pub = self.create_publisher(Float64, self.servo_topic, 10)

        time.sleep(0.3)  # allow publisher discovery, same as the other tuning nodes
        self.publish_stop()
        self.get_logger().info(
            f'speed_sweep_diagnostic_node up -- stop command published. '
            f'{len(self.target_speeds)} target(s): {self.target_speeds}')

    # ----- low-level publishing (same names/signatures as light_motion mode
    # in sensor_covariance_calibration_node.py, reimplemented standalone) ---
    def publish_drive(self, erpm: float, servo: float):
        self._speed_pub.publish(Float64(data=float(erpm)))
        self._servo_pub.publish(Float64(data=float(servo)))

    def publish_stop(self):
        self.publish_drive(0.0, self.steering_center)

    def _drive_for(self, erpm: float, duration_sec: float):
        n = int(duration_sec * PUBLISH_RATE_HZ)
        try:
            for _ in range(n):
                if not rclpy.ok():
                    break
                self.publish_drive(erpm, self.steering_center)
                time.sleep(1.0 / PUBLISH_RATE_HZ)
        finally:
            # Always stop, even if interrupted mid-run.
            self.publish_stop()

    # ----- interactive sweep -------------------------------------------
    def run_sweep(self):
        results = []  # list of (target_speed, actual_speed, abs_error, pct_error)
        for target in self.target_speeds:
            row = self._one_target(target)
            if row is None:
                self.get_logger().warning(
                    'Sweep aborted -- printing summary for targets completed so far.')
                break
            results.append(row)

        self._print_summary(results)
        if self.write_csv and results:
            self._write_csv(results)
        elif not results:
            print('No results collected -- nothing to summarize or write.')

    def _one_target(self, target: float):
        """Run one sweep point. Returns a result row, or None on abort ('q')."""
        print(SEP)
        print('SPEED SWEEP -- COMMANDED VS ACTUAL')
        print(f'  target speed   : {target:.3f} m/s')
        print(f'  drive duration : {self.drive_duration_sec:.1f} s')
        print(f'  expected dist. : ~{target * self.drive_duration_sec:.2f} m '
              f'(if commanded == actual)')
        print(SEP)
        raw = input(f'Press ENTER when ready to drive at {target:.2f} m/s for '
                    f'{self.drive_duration_sec:.1f}s, or \'q\' to abort: ').strip().lower()
        if raw == 'q':
            return None

        erpm = self.speed_to_erpm_gain * target + self.speed_to_erpm_offset
        print(f'  Driving: erpm={erpm:.1f} for {self.drive_duration_sec:.2f} s ...')
        self._drive_for(erpm, self.drive_duration_sec)
        print('Car stopped. Please measure the actual distance travelled in metres.')

        measured = self._read_measured_distance(target, self.drive_duration_sec)
        if measured is None:
            return None

        actual = measured / self.drive_duration_sec
        abs_error = abs(actual - target)
        pct_error = (abs_error / target * 100.0) if target else 0.0
        print(f'  -> target={target:.3f} m/s  actual={actual:.3f} m/s  '
              f'abs_error={abs_error:.3f} m/s  pct_error={pct_error:.2f}%\n')
        return (target, actual, abs_error, pct_error)

    def _read_measured_distance(self, target: float, duration: float):
        """Return a float distance (m), or None to abort. Re-asks on bad input,
        and requires an explicit y/n confirmation for oddly-large-looking values
        (typo guard) -- same pattern as speed_tuning_node.py's equivalent."""
        expected = target * duration
        sanity_ceiling = max(expected * 5.0, 0.5)
        while rclpy.ok():
            raw = input("Enter measured distance traveled in meters, or 'q' to "
                         "abort: ").strip().lower()
            if raw == 'q':
                return None
            try:
                value = float(raw)
            except ValueError:
                print('  Not a number -- try again.')
                continue
            if value <= 0:
                print('  Must be positive -- try again.')
                continue
            if value > sanity_ceiling:
                confirm = input(
                    f'  {value:.3f} m seems large for a {duration:.1f}s run at '
                    f'{target:.2f} m/s target (expected ~{expected:.2f} m). Use it '
                    f'anyway? (y/n): ').strip().lower()
                if confirm != 'y':
                    continue
            return value
        return None

    def _print_summary(self, results):
        if not results:
            return
        rows = sorted(results, key=lambda r: r[0])
        print(SEP)
        print('SPEED SWEEP SUMMARY (sorted by target speed)')
        print(SEP)
        print(f'{"Target (m/s)":>14}{"Actual (m/s)":>16}{"Abs Error (m/s)":>18}'
              f'{"Pct Error (%)":>16}')
        for target, actual, abs_error, pct_error in rows:
            print(f'{target:>14.3f}{actual:>16.3f}{abs_error:>18.3f}{pct_error:>16.2f}')
        print(SEP)

    def _write_csv(self, results):
        rows = sorted(results, key=lambda r: r[0])
        if self.csv_path:
            path = Path(self.csv_path)
        else:
            results_dir = _default_results_dir()
            results_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
            path = results_dir / f'speed_sweep_{timestamp}.csv'

        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['target_speed_mps', 'actual_speed_mps',
                                  'abs_error_mps', 'pct_error'])
                for row in rows:
                    writer.writerow([f'{v:.6f}' for v in row])
            print(f'Results written to: {path}')
        except OSError as exc:
            self.get_logger().error(f'Could not write CSV to {path}: {exc}')


def main():
    rclpy.init()
    node = SpeedSweepDiagnosticNode()
    try:
        node.run_sweep()
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        # SAFETY: always leave the car stopped, on every exit path.
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
