#!/usr/bin/env python3
"""Menu-driven steering calibration for the F1TENTH VESC stack: range/bottleneck
symmetry, offset, and gain -- three human-in-the-loop modes in one node.

Fully open-loop and human-in-the-loop, same philosophy as speed_tuning_node.py /
steering_tuning_node.py: no IMU, no odometry -- every measurement is physically
taken by a human (tape measure) and typed at the terminal. All driving in every
mode uses the same constant ``test_speed`` (default 0.5 m/s).

On startup you get a text menu:

    [1] range/bottleneck symmetry
    [2] offset
    [3] gain
    [q] quit

Each mode runs its own drive/measure/adjust loop and returns to this menu when
it exits (confirmed 'good'/'y', or aborted with 'q'), so any step can be
re-run without restarting the node.

Mode 1 -- RANGE / BOTTLENECK SYMMETRY
    Drives full lock left, then full lock right, each a full circle, prompting
    for the measured radius after each. The side with the LARGER radius steers
    less -- it is the mechanical bottleneck. The OTHER side's effective
    ``servo_min``/``servo_max`` (the same absolute-clamp parameters
    vesc_driver_node itself reads from vesc.yaml) is pulled toward
    ``steering_to_servo_offset`` until its angle matches the bottleneck, via
    the bicycle-model inverse ``delta = atan(wheelbase / R)``:

        scale        = delta_bottleneck / delta_other        (<= 1)
        new_extreme  = offset + (old_extreme - offset) * scale

    This adjusts the commanded RANGE, not steering_to_servo_gain_left/_right or
    steering_to_servo_offset -- those are left untouched by this mode
    (steering_to_servo_gain_left's sign is only read, to know which physical
    side 'left' is -- see _left_is_min()). Repeats (re-driving both sides
    against the new range) until you confirm it's good.

Mode 2 -- OFFSET
    Drives straight (servo = current offset) for a fixed, time-based distance
    (default 2 m -- no odometry/IMU is used, so distance = duration *
    test_speed). You type the measured lateral deviation from the intended
    line (positive = right, negative = left). The offset is corrected
    proportionally (``offset_correction_gain``, a node param so it can be
    retuned): a positive/right deviation subtracts from the offset, biasing
    toward the servo_min/left side, since steering_to_servo_gain_left is
    negative for this car (servo decreases for a left/positive-angle command
    -- see vesc.yaml). If your car's left gain has the opposite sign, or this
    makes the drift worse, negate offset_correction_gain. Repeats until
    confirmed good.

Mode 3 -- GAIN
    Computes the steering angle that should produce a ``gain_target_radius_m``
    (default 1 m) circle via the bicycle model (``delta = atan(wheelbase /
    R)``). LEFT and RIGHT each get their own fully independent sub-loop, run
    sequentially (LEFT first, fully converged/confirmed, then RIGHT) --
    steering_to_servo_gain_left and steering_to_servo_gain_right are separate
    node state, and each sub-loop only ever reads/writes its own gain. This
    replaces an earlier single-shared-gain design that oscillated, since one
    gain can't simultaneously fit both sides of an asymmetric linkage.

    Each sub-loop: drive the commanded circle, prompt for the measured
    radius, rescale that direction's gain by the measured/target ratio (same
    formula as steering_tuning_node.py's quarter-circle calibration):

        new_gain = old_gain * (measured_radius / target_radius)

    then either auto-continue if within ``gain_tolerance_m`` of the target
    (prints "converged"), or ask you to confirm ('good'/'y') or retest,
    same confirm/retry pattern as Modes 1/2 -- so you can stop early even if
    not strictly within tolerance, or keep iterating past it. Bounded by
    ``gain_max_iterations`` per side as a safety cap either way.

Startup and persistence: all 5 values (servo_min, servo_max,
steering_angle_to_servo_offset, steering_angle_to_servo_gain_left/_right) are
read directly from ``steering_calibration_yaml_path`` at startup -- NOT from
hardcoded defaults, and NOT from a possibly-stale running driver's ROS
params -- see ``_load_calibration()``. That file (defaults to
f1tenth_hardware/config/steering_calibration.yaml, the same file
vesc.launch.py points vesc_driver_node/ackermann_to_vesc_node at) is the one
source of truth this node and the real drivers both read. Startup fails
loudly (fatal log + exit) rather than silently falling back to a baked-in
default if the file is missing, unreadable, or missing a key -- driving on a
silently-stale default would be worse than not starting.

After EVERY confirmed/applied change to any of the 5 values -- every
iteration a value changes, not just when a mode's loop finally exits -- the
full current set of all 5 is atomically rewritten to
steering_calibration_yaml_path (temp file in the same directory, then
os.replace() -- see yaml_updater.atomic_write_calibration_yaml), so a crash
or Ctrl-C mid-write can't corrupt the file the real driver depends on. Each
such write also prints a reminder: neither vesc_driver_node nor
ackermann_to_vesc_node hot-reloads this file or supports `ros2 param set`
for these keys (both read all parameters once at startup; neither registers
a set-parameters callback -- confirmed by reading their source) -- restart
the real driver to pick up a change made here.

Threading / safety: identical pattern to speed_tuning_node.py /
steering_tuning_node.py -- the whole interactive session (menu + all three
modes, which use blocking ``input()``) runs on a worker thread; ``rclpy.spin``
runs on the main thread so the node stays alive and able to publish stops
while waiting on the keyboard. A stop is published on startup, after every
drive, and in every ``finally`` path. 'q' at any prompt aborts the current
mode back to the menu (car already stopped by the drive's own finally block).
Ctrl-C interrupts ``rclpy.spin`` on the main thread and shuts the node down
cleanly (stop published) -- like the two reference nodes, it cannot unblock
a pending ``input()`` on the worker thread, so it ends the process rather
than returning to the menu; this matches their existing behavior exactly.

Run with `ros2 run`, NOT `ros2 launch`: stdin is not reliably forwarded to
nodes started via `ros2 launch` (see speed_tuning_node.py's note). No launch
file is provided for this node for that reason.
"""

import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from vesc_tuning.yaml_updater import atomic_write_calibration_yaml, read_vesc_param

PUBLISH_RATE_HZ = 50.0
GAIN_MIN_WARN = 0.1   # magnitude
GAIN_MAX_WARN = 5.0   # magnitude
SEP = '=' * 70


class SteeringCalibrationNode(Node):

    def __init__(self):
        super().__init__('steering_calibration_node')

        # --- topics ---------------------------------------------------
        self.command_topic = str(
            self.declare_parameter('command_topic', '/commands/motor/speed').value)
        self.servo_topic = str(
            self.declare_parameter('servo_topic', '/commands/servo/position').value)

        # --- the 5 steering-calibration values: source of truth is this file,
        # not a hardcoded default and not a running node's ROS params -- see
        # _load_calibration(). Default path resolves to f1tenth_hardware's
        # installed steering_calibration.yaml (the same file vesc.launch.py
        # points the real drivers at); wrapped in try/except so a workspace
        # where f1tenth_hardware isn't built yet fails at _load_calibration()
        # (a clear fatal log) instead of crashing here on package lookup.
        try:
            default_calibration_yaml = os.path.join(
                get_package_share_directory('f1tenth_hardware'),
                'config', 'steering_calibration.yaml')
        except PackageNotFoundError:
            default_calibration_yaml = ''
        self.calibration_yaml_path = str(self.declare_parameter(
            'steering_calibration_yaml_path', default_calibration_yaml).value)
        # Sets self.servo_min, self.servo_max, self.steering_to_servo_offset,
        # self.steering_to_servo_gain_left, self.steering_to_servo_gain_right.
        # Raises RuntimeError (caught in main()) if the file can't be read.
        self._load_calibration()

        # --- everything else (unaffected by the split above -- still lives
        # in vesc.yaml, seeded the same way speed_tuning_node.py does) ------
        self.wheelbase = float(self.declare_parameter('wheelbase', 0.305).value)
        self.vesc_yaml_path = str(self.declare_parameter('vesc_yaml_path', '').value)
        self.speed_to_erpm_gain = float(
            self.declare_parameter('speed_to_erpm_gain', 4614.0).value)
        self.speed_to_erpm_offset = float(
            self.declare_parameter('speed_to_erpm_offset', 0.0).value)
        self._seed_from_yaml('speed_to_erpm_gain', 'speed_to_erpm_gain')
        self._seed_from_yaml('speed_to_erpm_offset', 'speed_to_erpm_offset')

        # --- driving: one constant speed, used by every mode -----------------
        self.test_speed = float(self.declare_parameter('test_speed', 0.5).value)
        self.drive_duration_safety_max = float(
            self.declare_parameter('drive_duration_safety_max', 20.0).value)
        self.circle_duration_margin = float(
            self.declare_parameter('circle_duration_margin', 1.15).value)

        # --- Mode 2 (offset) ---------------------------------------------
        self.offset_trial_distance_m = float(
            self.declare_parameter('offset_trial_distance_m', 2.0).value)
        # servo units per metre of measured lateral deviation -- see the
        # module docstring's Mode 2 section for the sign convention and when
        # to negate this.
        self.offset_correction_gain = float(
            self.declare_parameter('offset_correction_gain', 0.05).value)

        # --- Mode 3 (gain) ----------------------------------------------
        self.gain_target_radius_m = float(
            self.declare_parameter('gain_target_radius_m', 1.0).value)
        self.gain_tolerance_m = float(self.declare_parameter('gain_tolerance_m', 0.05).value)
        self.gain_max_iterations = int(self.declare_parameter('gain_max_iterations', 15).value)

        # --- Mode 1 (range/bottleneck) safety cap -------------------------
        self.range_max_iterations = int(
            self.declare_parameter('range_max_iterations', 15).value)

        # --- publishers ---------------------------------------------------
        self.speed_pub = self.create_publisher(Float64, self.command_topic, 10)
        self.servo_pub = self.create_publisher(Float64, self.servo_topic, 10)

        self._active = threading.Event()
        self._active.set()

        # Safety: publish a stop the moment the node comes up.
        time.sleep(0.3)  # allow publisher discovery
        self.publish_stop()
        self.get_logger().info('steering_calibration_node up -- stop command published.')

    def _seed_from_yaml(self, attr, param_name):
        """Override self.<attr> with the current value from vesc.yaml, if any
        (same pattern as speed_tuning_node.py / steering_tuning_node.py)."""
        value = read_vesc_param(self.vesc_yaml_path, param_name)
        if value is None:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        setattr(self, attr, value)
        self.get_logger().info(f'Starting {param_name} = {value} (from {self.vesc_yaml_path})')

    # ----- steering_calibration.yaml: the one source of truth for the 5 ----
    def _load_calibration(self):
        """Read all 5 steering-calibration values directly from
        calibration_yaml_path -- the same file the real vesc_driver_node/
        ackermann_to_vesc_node read at launch (see vesc.launch.py). Unlike
        _seed_from_yaml (a soft override on top of a hardcoded fallback
        default), this is the ONLY source: fatal + raise if the file is
        missing, unreadable, or any key is absent, rather than silently
        driving with a baked-in default that could be stale or simply wrong
        for this car."""
        key_for_attr = {
            'servo_min': 'servo_min',
            'servo_max': 'servo_max',
            'steering_to_servo_offset': 'steering_angle_to_servo_offset',
            'steering_to_servo_gain_left': 'steering_angle_to_servo_gain_left',
            'steering_to_servo_gain_right': 'steering_angle_to_servo_gain_right',
        }
        missing = []
        for attr, yaml_key in key_for_attr.items():
            value = read_vesc_param(self.calibration_yaml_path, yaml_key)
            if value is None:
                missing.append(yaml_key)
                continue
            try:
                setattr(self, attr, float(value))
            except (TypeError, ValueError):
                missing.append(yaml_key)
        if missing:
            self.get_logger().fatal(
                f'Could not read {missing} from steering_calibration_yaml_path='
                f"'{self.calibration_yaml_path}' -- file missing, unreadable, or "
                'those keys absent. Refusing to start with stale/hardcoded '
                'steering calibration values.')
            raise RuntimeError(
                f'failed to load steering calibration from {self.calibration_yaml_path}')
        self.get_logger().info(
            f'Loaded steering calibration from {self.calibration_yaml_path}: '
            f'servo_min={self.servo_min:.4f} servo_max={self.servo_max:.4f} '
            f'offset={self.steering_to_servo_offset:.4f} '
            f'gain_left={self.steering_to_servo_gain_left:.4f} '
            f'gain_right={self.steering_to_servo_gain_right:.4f}')

    def _save_calibration(self):
        """Persist the full current 5-value set to calibration_yaml_path,
        atomically (see yaml_updater.atomic_write_calibration_yaml). Called
        by every setter that touches one of these 5 values (_set_offset,
        _set_servo_extreme, _set_gain) -- so the file the real driver reads
        from is always caught up within one write of any confirmed change,
        not just at the end of a mode's loop."""
        values = {
            'servo_min': self.servo_min,
            'servo_max': self.servo_max,
            'steering_angle_to_servo_offset': self.steering_to_servo_offset,
            'steering_angle_to_servo_gain_left': self.steering_to_servo_gain_left,
            'steering_angle_to_servo_gain_right': self.steering_to_servo_gain_right,
        }
        try:
            atomic_write_calibration_yaml(self.calibration_yaml_path, values)
        except OSError as exc:
            print(f'  [ERROR] could not write {self.calibration_yaml_path}: {exc} -- '
                  f'in-memory value updated, but NOT persisted to disk.')
            return
        print(f'  Saved to {self.calibration_yaml_path}.')
        print("  NOTE: the real vesc driver/ackermann_to_vesc node (if already running) "
              'reads its params once at startup -- it will NOT pick this up live, and '
              '`ros2 param set` will not help either (neither node registers a '
              'set-parameters callback). Restart it to apply.')

    # ----- low-level publishing ------------------------------------------
    def _pub_speed(self, erpm):
        msg = Float64()
        msg.data = float(erpm)
        self.speed_pub.publish(msg)

    def _pub_servo(self, value):
        msg = Float64()
        msg.data = float(value)
        self.servo_pub.publish(msg)

    def publish_stop(self):
        """Center the servo (current offset) and command zero motor speed."""
        self._pub_servo(self.steering_to_servo_offset)
        self._pub_speed(0.0)

    def _drive(self, servo_cmd, duration):
        """Hold servo_cmd and command test_speed (converted to erpm) at 50 Hz
        for duration seconds. Returns False if aborted mid-drive (Ctrl-C /
        self._active cleared); always ends by publishing a stop, same pattern
        as speed_tuning_node.drive_straight / steering_tuning_node.drive_arc."""
        erpm = self.speed_to_erpm_gain * self.test_speed + self.speed_to_erpm_offset
        # SAFETY: steering is set before any motor command.
        self._pub_servo(servo_cmd)
        n = int(duration * PUBLISH_RATE_HZ)
        aborted = False
        try:
            for _ in range(n):
                if not self._active.is_set() or not rclpy.ok():
                    aborted = True
                    break
                self._pub_servo(servo_cmd)
                self._pub_speed(erpm)
                time.sleep(1.0 / PUBLISH_RATE_HZ)
        finally:
            self.publish_stop()
        return not aborted

    def request_stop(self):
        self._active.clear()

    # ----- generic prompt helpers ------------------------------------------
    def _read_float(self, prompt, min_value=None, max_value=None):
        """Blocking float prompt; 'q' aborts (returns None). Retries on bad
        input or out-of-range values (same pattern as
        speed_tuning_node._read_measured_distance)."""
        while self._active.is_set() and rclpy.ok():
            raw = input(prompt).strip().lower()
            if raw == 'q':
                return None
            try:
                value = float(raw)
            except ValueError:
                print('  Not a number -- try again.')
                continue
            if min_value is not None and value < min_value:
                print(f'  Must be >= {min_value} -- try again.')
                continue
            if max_value is not None and value > max_value:
                print(f'  Must be <= {max_value} -- try again.')
                continue
            return value
        return None

    @staticmethod
    def _ask_confirm_or_retry(prompt):
        """Returns 'confirm', 'retry', or 'quit'."""
        raw = input(prompt).strip().lower()
        if raw == 'q':
            return 'quit'
        if raw in ('good', 'y', 'yes'):
            return 'confirm'
        return 'retry'

    # ----- direction / servo-range helpers ---------------------------------
    def _left_is_min(self):
        """True if the servo_min side is 'left'. Ackermann convention:
        positive commanded angle = left (matches steering_tuning_node.py's
        direction handling, and ackermann_to_vesc_node's own
        steering_angle >= 0 -> gain_left branch). Picked by the sign of
        steering_to_servo_gain_left specifically -- that's the gain applied
        to left (>=0) commanded angles, so its sign alone determines which
        physical side 'left' throws toward, independent of gain_right."""
        return self.steering_to_servo_gain_left < 0

    def _servo_extreme(self, direction):
        """Current effective full-lock servo command for 'left' / 'right'."""
        if direction == 'left':
            return self.servo_min if self._left_is_min() else self.servo_max
        return self.servo_max if self._left_is_min() else self.servo_min

    def _set_servo_extreme(self, direction, value):
        """Mutate servo_min or servo_max (whichever `direction` maps to) and
        immediately persist -- see _save_calibration()."""
        if (direction == 'left') == self._left_is_min():
            self.servo_min = value
        else:
            self.servo_max = value
        self._save_calibration()

    def _set_offset(self, value):
        """Mutate steering_to_servo_offset and immediately persist."""
        self.steering_to_servo_offset = value
        self._save_calibration()

    def _get_gain(self, direction):
        return self.steering_to_servo_gain_left if direction == 'left' \
            else self.steering_to_servo_gain_right

    def _set_gain(self, direction, value):
        """Mutate gain_left or gain_right (never both) and immediately
        persist -- see _save_calibration()."""
        if direction == 'left':
            self.steering_to_servo_gain_left = value
        else:
            self.steering_to_servo_gain_right = value
        self._save_calibration()

    def _estimate_radius(self, servo_cmd, direction):
        """Rough radius estimate from the current (possibly untuned)
        per-direction gain / offset, used only to size a full-circle drive's
        duration -- never for calibration math itself. Uses direction's own
        gain (not the other side's) to predict the servo command's angle."""
        gain = self._get_gain(direction)
        if not gain:
            return self.wheelbase * 10.0
        angle_est = abs((servo_cmd - self.steering_to_servo_offset) / gain)
        if angle_est < 1e-4:
            return self.wheelbase * 10.0
        return self.wheelbase / math.tan(angle_est)

    def _circle_duration(self, radius_m):
        """Time to trace one full circle of radius_m at test_speed, with a
        margin so the loop closes past its start, capped for safety."""
        if self.test_speed <= 0 or radius_m <= 0:
            return 0.0
        duration = (2.0 * math.pi * radius_m / self.test_speed) * self.circle_duration_margin
        if duration > self.drive_duration_safety_max:
            print(f'  [safety] run time capped at {self.drive_duration_safety_max:.1f} s '
                  f'(was {duration:.1f} s).')
            duration = self.drive_duration_safety_max
        return duration

    # ----- MODE 1: range / bottleneck symmetry ------------------------------
    def _run_range_mode(self):
        print(SEP)
        print('MODE 1: RANGE / BOTTLENECK SYMMETRY')
        print('  Drives full lock left, then full lock right, each a full circle.')
        print("  The side with the LARGER measured radius (steers less) is the")
        print("  bottleneck; the other side's effective servo extreme is pulled")
        print('  toward center until its angle matches the bottleneck.')
        print(f'  Starting range: servo_min={self.servo_min:.4f}  servo_max={self.servo_max:.4f}')
        print(SEP)

        iteration = 0
        while self._active.is_set() and rclpy.ok():
            iteration += 1
            radius_left = self._run_one_lock('left')
            if radius_left is None:
                self._print_final_values('MODE 1 aborted -- current values:')
                return
            radius_right = self._run_one_lock('right')
            if radius_right is None:
                self._print_final_values('MODE 1 aborted -- current values:')
                return

            if radius_left > radius_right:
                bottleneck, other, r_bottleneck, r_other = 'left', 'right', radius_left, radius_right
            elif radius_right > radius_left:
                bottleneck, other, r_bottleneck, r_other = 'right', 'left', radius_right, radius_left
            else:
                bottleneck = other = None
                print('  Radii already match -- range is symmetric, nothing to adjust.')

            if other is not None:
                delta_bottleneck = math.atan(self.wheelbase / r_bottleneck)
                delta_other = math.atan(self.wheelbase / r_other)
                scale = (delta_bottleneck / delta_other) if delta_other else 1.0

                old_extreme = self._servo_extreme(other)
                throw = old_extreme - self.steering_to_servo_offset
                new_extreme = self.steering_to_servo_offset + throw * scale
                new_extreme = max(0.0, min(1.0, new_extreme))

                print(f'  {bottleneck.upper()} is the bottleneck (R={r_bottleneck:.3f} m, '
                      f'delta={delta_bottleneck:.4f} rad).')
                print(f'  Pulling in {other.upper()} extreme: {old_extreme:.4f} -> '
                      f'{new_extreme:.4f}  (delta {delta_other:.4f} -> {delta_bottleneck:.4f} rad, '
                      f'scale={scale:.4f})')
                self._set_servo_extreme(other, new_extreme)

            print(f'  Effective range now: servo_min={self.servo_min:.4f}  '
                  f'servo_max={self.servo_max:.4f}')

            if iteration >= self.range_max_iterations:
                print(f'  [safety] max_iterations ({self.range_max_iterations}) reached -- '
                      f'stopping.')
                break

            outcome = self._ask_confirm_or_retry(
                "Type 'good' or 'y' to accept this range, anything else to re-test, "
                "or 'q' to abort: ")
            if outcome == 'quit':
                self._print_final_values('MODE 1 aborted -- current values:')
                return
            if outcome == 'confirm':
                break

        self._print_final_values('MODE 1 finished -- final calibrated values:')

    def _run_one_lock(self, direction):
        """Drive a full circle at full lock `direction`. Returns the measured
        radius, or None on abort ('q' or Ctrl-C)."""
        servo_cmd = self._servo_extreme(direction)
        duration = self._circle_duration(self._estimate_radius(servo_cmd, direction))

        raw = input(f'Press ENTER to drive FULL LOCK {direction.upper()} '
                    f'(servo={servo_cmd:.4f}) at {self.test_speed:.2f} m/s for '
                    f'~{duration:.1f} s (full circle), or "q" to abort: ').strip().lower()
        if raw == 'q':
            return None
        print(f'  Driving: servo={servo_cmd:.4f} for {duration:.2f} s ...')
        if not self._drive(servo_cmd, duration):
            return None
        print('Car stopped.')
        return self._read_float('Enter measured circle radius (m): ', min_value=0.01)

    # ----- MODE 2: offset ---------------------------------------------------
    def _run_offset_mode(self):
        print(SEP)
        print('MODE 2: OFFSET')
        print(f'  Drives straight ({self.offset_trial_distance_m:.2f} m at '
              f'{self.test_speed:.2f} m/s, time-based -- no odometry/IMU is used).')
        print(SEP)

        duration = self.offset_trial_distance_m / self.test_speed if self.test_speed else 0.0
        if duration > self.drive_duration_safety_max:
            print(f'  [safety] run time capped at {self.drive_duration_safety_max:.1f} s.')
            duration = self.drive_duration_safety_max

        while self._active.is_set() and rclpy.ok():
            print(f'Current steering_angle_to_servo_offset = {self.steering_to_servo_offset:.4f}')
            raw = input(f'Press ENTER to drive straight {self.offset_trial_distance_m:.2f} m, '
                        f'or "q" to abort: ').strip().lower()
            if raw == 'q':
                self._print_final_values('MODE 2 aborted -- current values:')
                return

            print(f'  Driving: servo={self.steering_to_servo_offset:.4f} for {duration:.2f} s ...')
            if not self._drive(self.steering_to_servo_offset, duration):
                self._print_final_values('MODE 2 aborted -- current values:')
                return
            print('Car stopped.')

            deviation = self._read_float(
                'Enter deviation from centerline in meters (positive = right, '
                'negative = left): ')
            if deviation is None:
                self._print_final_values('MODE 2 aborted -- current values:')
                return

            new_offset = self.steering_to_servo_offset - self.offset_correction_gain * deviation
            new_offset = max(self.servo_min, min(self.servo_max, new_offset))
            print(f'  Old offset: {self.steering_to_servo_offset:.4f}  ->  '
                  f'New offset: {new_offset:.4f}')
            self._set_offset(new_offset)

            outcome = self._ask_confirm_or_retry(
                "Type 'good' or 'y' to accept this offset, anything else to retest, "
                "or 'q' to abort: ")
            if outcome == 'quit':
                self._print_final_values('MODE 2 aborted -- current values:')
                return
            if outcome == 'confirm':
                break

        self._print_final_values('MODE 2 finished -- final calibrated values:')

    # ----- MODE 3: gain (LEFT and RIGHT fully independent, sequential) -------
    def _run_gain_mode(self):
        print(SEP)
        print('MODE 3: GAIN')
        target_angle = math.atan(self.wheelbase / self.gain_target_radius_m)
        print(f'  Target: {self.gain_target_radius_m:.2f} m radius circle -> commanded angle '
              f'{target_angle:.4f} rad (bicycle model, wheelbase={self.wheelbase:.3f} m)')
        print(f'  tolerance = +/-{self.gain_tolerance_m:.3f} m, '
              f'max_iterations={self.gain_max_iterations} per side')
        print('  LEFT and RIGHT are tuned independently, one fully converged/confirmed')
        print('  at a time -- each sub-loop only ever touches its own gain.')
        print(SEP)

        if not self._run_gain_side('left', target_angle):
            self._print_final_values('MODE 3 aborted -- current values:')
            return
        if not self._run_gain_side('right', target_angle):
            self._print_final_values('MODE 3 aborted -- current values:')
            return

        self._print_final_values('MODE 3 finished -- final calibrated values:')

    def _run_gain_side(self, direction, target_angle):
        """Fully converge (or manually accept) the gain for one direction
        only -- never reads or writes the other direction's gain. Returns
        False on abort ('q' / Ctrl-C), True once confirmed/converged/capped."""
        print(f'--- {direction.upper()} ---')
        iteration = 0
        while self._active.is_set() and rclpy.ok():
            iteration += 1
            print(f'  [{direction}] iteration {iteration}, current gain = '
                  f'{self._get_gain(direction):.4f}')

            radius = self._run_one_gain_circle(direction, target_angle)
            if radius is None:
                return False
            self._update_gain(direction, radius)

            err = abs(radius - self.gain_target_radius_m)
            print(f'  [{direction}] measured R={radius:.3f} m (err {err:+.3f} m), '
                  f'gain={self._get_gain(direction):.4f}')

            if err <= self.gain_tolerance_m:
                print(f'  [{direction}] converged -- within tolerance.')
                return True
            if iteration >= self.gain_max_iterations:
                print(f'  [{direction}] [safety] max_iterations '
                      f'({self.gain_max_iterations}) reached -- stopping.')
                return True

            outcome = self._ask_confirm_or_retry(
                f"[{direction}] Type 'good' or 'y' to accept this gain, anything else "
                f"to retest, or 'q' to abort: ")
            if outcome == 'quit':
                return False
            if outcome == 'confirm':
                return True
        return False

    def _run_one_gain_circle(self, direction, target_angle):
        signed_angle = target_angle if direction == 'left' else -target_angle
        gain = self._get_gain(direction)
        servo_cmd = gain * signed_angle + self.steering_to_servo_offset
        servo_cmd = max(self.servo_min, min(self.servo_max, servo_cmd))
        duration = self._circle_duration(self.gain_target_radius_m)

        raw = input(f'Press ENTER to drive {direction.upper()} circle (servo={servo_cmd:.4f}) '
                    f'at {self.test_speed:.2f} m/s for ~{duration:.1f} s, '
                    f'or "q" to abort: ').strip().lower()
        if raw == 'q':
            return None
        print(f'  Driving: servo={servo_cmd:.4f} for {duration:.2f} s ...')
        if not self._drive(servo_cmd, duration):
            return None
        print('Car stopped.')
        return self._read_float('Enter measured circle radius (m): ', min_value=0.01)

    def _update_gain(self, direction, measured_radius):
        old_gain = self._get_gain(direction)
        new_gain = old_gain * (measured_radius / self.gain_target_radius_m)
        if abs(new_gain) < GAIN_MIN_WARN or abs(new_gain) > GAIN_MAX_WARN:
            print(f'  [{direction}] [warning] new gain magnitude {abs(new_gain):.3f} is '
                  f'outside [{GAIN_MIN_WARN}, {GAIN_MAX_WARN}] -- check the measurement.')
        print(f'  [{direction}] Old gain: {old_gain:.4f}  ->  New gain: {new_gain:.4f}')
        self._set_gain(direction, new_gain)

    # ----- shared printing ---------------------------------------------------
    def _print_final_values(self, header):
        print(SEP)
        print(header)
        print(f'  servo_min                          : {self.servo_min:.4f}')
        print(f'  servo_max                          : {self.servo_max:.4f}')
        print(f'  steering_angle_to_servo_offset     : {self.steering_to_servo_offset:.4f}')
        print(f'  steering_angle_to_servo_gain_left  : {self.steering_to_servo_gain_left:.4f}')
        print(f'  steering_angle_to_servo_gain_right : {self.steering_to_servo_gain_right:.4f}')
        print(f'  (already saved to {self.calibration_yaml_path} -- restart the real '
              'vesc driver/ackermann_to_vesc node to apply.)')
        print(SEP + '\n')

    def _print_menu(self):
        print(SEP)
        print('STEERING CALIBRATION NODE')
        print(f'  test_speed (constant, all modes)   : {self.test_speed:.2f} m/s')
        print(f'  servo_min / servo_max              : {self.servo_min:.4f} / {self.servo_max:.4f}')
        print(f'  steering_angle_to_servo_offset     : {self.steering_to_servo_offset:.4f}')
        print(f'  steering_angle_to_servo_gain_left  : {self.steering_to_servo_gain_left:.4f}')
        print(f'  steering_angle_to_servo_gain_right : {self.steering_to_servo_gain_right:.4f}')
        print(SEP)

    # ----- top-level menu loop (worker thread) -------------------------------
    def run_interactive(self):
        try:
            self.publish_stop()
            while self._active.is_set() and rclpy.ok():
                self._print_menu()
                choice = input('Select mode  [1] range/bottleneck  [2] offset  '
                                '[3] gain  [q] quit: ').strip().lower()
                if choice == 'q':
                    break
                elif choice == '1':
                    self._run_range_mode()
                elif choice == '2':
                    self._run_offset_mode()
                elif choice == '3':
                    self._run_gain_mode()
                else:
                    print('  Not a valid choice -- enter 1, 2, 3, or q.\n')
        finally:
            self.publish_stop()
            # break the main-thread spin so the process can exit on 'q'
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SteeringCalibrationNode()
    except RuntimeError:
        # steering_calibration.yaml missing/unreadable -- already fatal-logged
        # in _load_calibration().
        if rclpy.ok():
            rclpy.shutdown()
        return

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
