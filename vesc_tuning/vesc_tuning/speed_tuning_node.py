#!/usr/bin/env python3
"""Empirical calibration of ``speed_to_erpm_gain`` by driving a straight line.

Human-in-the-loop pattern:
    command -> car moves -> human measures distance -> human types result ->
    node recomputes gain -> writes vesc.yaml -> prompts to restart -> loops.

The interactive loop (which uses blocking ``input()``) runs in a worker thread;
``rclpy.spin`` runs on the main thread. This keeps the node alive and able to
publish stop commands while waiting for keyboard input. A stop command is
published on startup and in every ``finally`` path so the car is always safe.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from vesc_tuning.yaml_updater import update_vesc_yaml

PUBLISH_RATE_HZ = 50.0
GAIN_MIN_WARN = 500.0
GAIN_MAX_WARN = 20000.0
SEP = '=' * 70


class SpeedTuningNode(Node):

    def __init__(self):
        super().__init__('speed_tuning_node')

        # --- parameters ---------------------------------------------------
        self.target_distance = float(self.declare_parameter('target_distance', 1.0).value)
        self.target_speed = float(self.declare_parameter('target_speed', 0.5).value)
        self.reverse = bool(self.declare_parameter('reverse', False).value)
        self.speed_to_erpm_gain = float(self.declare_parameter('speed_to_erpm_gain', 4614.0).value)
        self.speed_to_erpm_offset = float(self.declare_parameter('speed_to_erpm_offset', 0.0).value)
        self.vesc_yaml_path = str(self.declare_parameter('vesc_yaml_path', '').value)
        self.command_topic = str(self.declare_parameter('command_topic', '/commands/motor/speed').value)
        self.servo_topic = str(self.declare_parameter('servo_topic', '/commands/servo/position').value)
        self.servo_center = float(self.declare_parameter('servo_center', 0.5304).value)
        self.drive_duration_safety_max = float(
            self.declare_parameter('drive_duration_safety_max', 10.0).value)

        # --- publishers ---------------------------------------------------
        self.speed_pub = self.create_publisher(Float64, self.command_topic, 10)
        self.servo_pub = self.create_publisher(Float64, self.servo_topic, 10)

        self._active = threading.Event()
        self._active.set()

        # Safety: publish a stop the moment the node comes up, before any run,
        # so launching the node alone (or hitting ENTER immediately) is safe.
        time.sleep(0.3)  # allow publisher discovery
        self.publish_stop()
        self.get_logger().info('speed_tuning_node up — stop command published.')

    # ----- low-level publishing ------------------------------------------
    def _pub_speed(self, erpm: float):
        msg = Float64()
        msg.data = float(erpm)
        self.speed_pub.publish(msg)

    def _pub_servo(self, value: float):
        msg = Float64()
        msg.data = float(value)
        self.servo_pub.publish(msg)

    def publish_stop(self):
        """Center the servo and command zero motor speed."""
        self._pub_servo(self.servo_center)
        self._pub_speed(0.0)

    def drive_straight(self, erpm: float, duration: float):
        """Hold servo center and command ``erpm`` at 50 Hz for ``duration`` s."""
        # SAFETY: servo center is published *before* (and alongside) any speed.
        self._pub_servo(self.servo_center)
        n = int(duration * PUBLISH_RATE_HZ)
        try:
            for _ in range(n):
                if not self._active.is_set() or not rclpy.ok():
                    break
                self._pub_servo(self.servo_center)
                self._pub_speed(erpm)
                time.sleep(1.0 / PUBLISH_RATE_HZ)
        finally:
            # Always stop, even if interrupted mid-run.
            self.publish_stop()

    def request_stop(self):
        self._active.clear()

    # ----- interactive loop (runs in worker thread) ----------------------
    def run_interactive(self):
        try:
            self.publish_stop()
            while self._active.is_set() and rclpy.ok():
                if not self._one_run():
                    break
        finally:
            self.publish_stop()
            # break the main-thread spin so the process can exit on 'q'
            if rclpy.ok():
                rclpy.shutdown()

    def _one_run(self) -> bool:
        """One tuning iteration. Returns False to quit the loop."""
        direction = 'REVERSE' if self.reverse else 'FORWARD'
        print(SEP)
        print('SPEED GAIN TUNING')
        print(f'  current speed_to_erpm_gain : {self.speed_to_erpm_gain:.3f}')
        print(f'  speed_to_erpm_offset       : {self.speed_to_erpm_offset:.3f}')
        print(f'  target_distance            : {self.target_distance:.3f} m')
        print(f'  target_speed               : {self.target_speed:.3f} m/s')
        print(f'  direction                  : {direction}')
        print(SEP)
        start = input(f'Press ENTER to drive {self.target_distance:.2f} m at '
                      f'{self.target_speed:.2f} m/s (make sure the path is clear), '
                      f'or "q" to quit: ').strip().lower()
        if start == 'q':
            return False

        # --- execute the run ---
        sign = -1.0 if self.reverse else 1.0
        speed_mps = self.target_speed * sign
        erpm = self.speed_to_erpm_gain * speed_mps + self.speed_to_erpm_offset
        duration = abs(self.target_distance / self.target_speed) if self.target_speed else 0.0
        if duration > self.drive_duration_safety_max:
            print(f'  [safety] run time capped at {self.drive_duration_safety_max:.1f} s '
                  f'(was {duration:.1f} s).')
            duration = self.drive_duration_safety_max

        print(f'  Driving: erpm={erpm:.1f} for {duration:.2f} s ...')
        self.drive_straight(erpm, duration)
        print('Car stopped. Please measure the actual distance travelled in metres.')

        # --- read measured distance ---
        measured = self._read_measured_distance()
        if measured is None:        # 'q'
            return False
        if measured == 'skip':      # 's'
            print('Skipped — gain unchanged.\n')
            return True

        # --- recompute gain ---
        # old_gain produced `measured` over the same drive time; to get
        # target_distance instead: new_gain = old_gain * target / measured.
        new_gain = self.speed_to_erpm_gain * (self.target_distance / measured)
        delta = new_gain - self.speed_to_erpm_gain
        pct = (delta / self.speed_to_erpm_gain * 100.0) if self.speed_to_erpm_gain else 0.0
        print(f'Old gain: {self.speed_to_erpm_gain:.3f}  ->  New gain: {new_gain:.3f}  '
              f'(change: {delta:+.3f}, {pct:+.2f}%)')

        if new_gain < GAIN_MIN_WARN or new_gain > GAIN_MAX_WARN:
            warn = input(f'  [warning] new gain {new_gain:.1f} is outside the sane range '
                         f'[{GAIN_MIN_WARN:.0f}, {GAIN_MAX_WARN:.0f}]. Apply anyway? (y/n): ')
            if warn.strip().lower() != 'y':
                print('Not applied.\n')
                return True

        if input('Apply new gain? (y/n): ').strip().lower() != 'y':
            print('Kept current gain.\n')
            return True

        self.speed_to_erpm_gain = new_gain
        self._write_gain(new_gain)
        print(SEP + '\n')
        return True

    def _read_measured_distance(self):
        """Return float distance, None to quit, or 'skip'. Re-asks on bad input."""
        while self._active.is_set() and rclpy.ok():
            raw = input("Enter measured distance (m), or 'q' to quit, "
                        "or 's' to skip this run: ").strip().lower()
            if raw == 'q':
                return None
            if raw == 's':
                return 'skip'
            try:
                value = float(raw)
            except ValueError:
                print('  Not a number — try again.')
                continue
            if value <= 0 or value > self.target_distance * 5:
                print(f'  Out of range (0, {self.target_distance * 5:.2f}] m — try again.')
                continue
            return value
        return None

    def _write_gain(self, new_gain: float):
        if self.vesc_yaml_path:
            import os
            if os.path.isfile(self.vesc_yaml_path):
                if update_vesc_yaml(self.vesc_yaml_path, 'speed_to_erpm_gain', new_gain):
                    print(f'Updated vesc.yaml at {self.vesc_yaml_path}. '
                          f'Restart the vesc nodes to apply.')
                else:
                    print('YAML update FAILED — set the value manually (see below).')
                    self._print_manual(new_gain)
                return
        print('vesc_yaml_path not set — gain updated in memory only.')
        self._print_manual(new_gain)

    @staticmethod
    def _print_manual(new_gain: float):
        print(f'Manually set  speed_to_erpm_gain: {new_gain:.3f}  in vesc.yaml and restart.')


def main(args=None):
    rclpy.init(args=args)
    node = SpeedTuningNode()

    worker = threading.Thread(target=node.run_interactive, daemon=True)
    worker.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.request_stop()
        # SAFETY: always leave the car stopped.
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
