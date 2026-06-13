#!/usr/bin/env python3
"""Empirical calibration of ``steering_angle_to_servo_gain`` via a quarter circle.

Bicycle model: a constant steering angle ``delta`` traces a circle of radius
``R = wheelbase / tan(delta)``. The car starts at (0, 0) heading +X. After a
perfect 90 deg arc the final position (relative to start) is:
    left turn  -> ( R,  R)
    right turn -> ( R, -R)

The human measures the actual final (x, y); the node computes::
    R_actual  = sqrt(x^2 + y^2) / sqrt(2)
    error_pct = (R_actual - R) / R * 100
    new_gain  = steering_to_servo_gain * (R_actual / R)

Threading / safety identical to speed_tuning_node: interactive loop (blocking
``input()``) on a worker thread, ``rclpy.spin`` on the main thread, stop command
on startup and in every ``finally`` path.

Note: ``speed_to_erpm_gain`` / ``speed_to_erpm_offset`` are also parameters here
because the arc is driven at ``target_speed`` (m/s) and the speed command topic
expects eRPM; they are NOT tuned by this node.
"""

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from vesc_tuning.yaml_updater import update_vesc_yaml

PUBLISH_RATE_HZ = 50.0
GAIN_MIN_WARN = 0.1   # magnitude
GAIN_MAX_WARN = 5.0   # magnitude
SEP = '=' * 70


class SteeringTuningNode(Node):

    def __init__(self):
        super().__init__('steering_tuning_node')

        # --- parameters ---------------------------------------------------
        self.steering_angle = float(self.declare_parameter('steering_angle', 0.26).value)
        self.target_speed = float(self.declare_parameter('target_speed', 0.5).value)
        self.direction = str(self.declare_parameter('direction', 'left').value).strip().lower()
        self.steering_to_servo_gain = float(
            self.declare_parameter('steering_to_servo_gain', -1.2135).value)
        self.steering_to_servo_offset = float(
            self.declare_parameter('steering_to_servo_offset', 0.5304).value)
        self.wheelbase = float(self.declare_parameter('wheelbase', 0.25).value)
        self.vesc_yaml_path = str(self.declare_parameter('vesc_yaml_path', '').value)
        self.command_topic = str(self.declare_parameter('command_topic', '/commands/motor/speed').value)
        self.servo_topic = str(self.declare_parameter('servo_topic', '/commands/servo/position').value)
        self.drive_duration_safety_max = float(
            self.declare_parameter('drive_duration_safety_max', 15.0).value)
        # Needed to convert target_speed (m/s) -> eRPM for the speed command.
        self.speed_to_erpm_gain = float(self.declare_parameter('speed_to_erpm_gain', 4614.0).value)
        self.speed_to_erpm_offset = float(self.declare_parameter('speed_to_erpm_offset', 0.0).value)

        # --- validate direction (error + exit if invalid) -----------------
        if self.direction not in ('left', 'right'):
            self.get_logger().fatal(
                f"Invalid 'direction' = '{self.direction}'. Must be 'left' or 'right'.")
            raise ValueError("direction must be 'left' or 'right'")

        # --- publishers ---------------------------------------------------
        self.speed_pub = self.create_publisher(Float64, self.command_topic, 10)
        self.servo_pub = self.create_publisher(Float64, self.servo_topic, 10)

        self._active = threading.Event()
        self._active.set()

        time.sleep(0.3)  # publisher discovery
        self.publish_stop()
        self.get_logger().info('steering_tuning_node up — stop command published.')

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
        """Center the servo (straight) and command zero motor speed."""
        self._pub_servo(self.steering_to_servo_offset)
        self._pub_speed(0.0)

    def drive_arc(self, erpm: float, servo_cmd: float, duration: float):
        """Hold ``servo_cmd`` and command ``erpm`` at 50 Hz for ``duration`` s."""
        # SAFETY: set the steering before commanding motion.
        self._pub_servo(servo_cmd)
        n = int(duration * PUBLISH_RATE_HZ)
        try:
            for _ in range(n):
                if not self._active.is_set() or not rclpy.ok():
                    break
                self._pub_servo(servo_cmd)
                self._pub_speed(erpm)
                time.sleep(1.0 / PUBLISH_RATE_HZ)
        finally:
            self.publish_stop()

    def request_stop(self):
        self._active.clear()

    # ----- interactive loop (worker thread) ------------------------------
    def run_interactive(self):
        try:
            self.publish_stop()
            while self._active.is_set() and rclpy.ok():
                if not self._one_run():
                    break
        finally:
            self.publish_stop()
            if rclpy.ok():
                rclpy.shutdown()

    def _one_run(self) -> bool:
        angle = abs(self.steering_angle)
        if angle <= 0.0:
            self.get_logger().fatal('steering_angle must be > 0.')
            return False

        radius = self.wheelbase / math.tan(angle)
        arc_length = radius * math.pi / 2.0
        duration = arc_length / self.target_speed if self.target_speed else 0.0
        if duration > self.drive_duration_safety_max:
            print(f'  [safety] run time capped at {self.drive_duration_safety_max:.1f} s '
                  f'(was {duration:.1f} s).')
            duration = self.drive_duration_safety_max

        signed_angle = angle if self.direction == 'left' else -angle
        servo_cmd = self.steering_to_servo_gain * signed_angle + self.steering_to_servo_offset
        erpm = self.speed_to_erpm_gain * self.target_speed + self.speed_to_erpm_offset
        exp_y = radius if self.direction == 'left' else -radius

        print(SEP)
        print('STEERING GAIN TUNING (quarter circle)')
        print(f'  current steering_to_servo_gain : {self.steering_to_servo_gain:.4f}')
        print(f'  steering_to_servo_offset       : {self.steering_to_servo_offset:.4f}')
        print(f'  steering_angle                 : {angle:.4f} rad  ({self.direction})')
        print(f'  wheelbase                      : {self.wheelbase:.3f} m')
        print(f'  expected radius R              : {radius:.3f} m')
        print(f'  expected arc length            : {arc_length:.3f} m')
        print(f'  target_speed                   : {self.target_speed:.3f} m/s')
        print(f'  expected run duration          : {duration:.2f} s')
        print(f'  expected final position        : x={radius:.3f}, y={exp_y:.3f} (m, rel. start)')
        print(f'  servo command                  : {servo_cmd:.4f}')
        print(SEP)

        start = input('Press ENTER to drive a quarter circle (path clear?), '
                      'or "q" to quit: ').strip().lower()
        if start == 'q':
            return False

        print(f'  Driving arc: erpm={erpm:.1f}, servo={servo_cmd:.4f} for {duration:.2f} s ...')
        self.drive_arc(erpm, servo_cmd, duration)
        print('Car stopped.')
        print(f'Expected final position: x={radius:.3f}, y={exp_y:.3f} (relative to start, m).')
        print('Measure the ACTUAL final position (x, y) relative to start.')

        coords = self._read_xy(radius)
        if coords is None:        # 'q'
            return False
        if coords == 'skip':      # 's'
            print('Skipped — gain unchanged.\n')
            return True
        x_actual, y_actual = coords

        r_actual = math.sqrt(x_actual ** 2 + y_actual ** 2) / math.sqrt(2.0)
        error_pct = (r_actual - radius) / radius * 100.0
        new_gain = self.steering_to_servo_gain * (r_actual / radius)

        print('-' * 70)
        print(f'  Expected R : {radius:.4f} m')
        print(f'  Actual R   : {r_actual:.4f} m   ({error_pct:+.2f}% error)')
        print(f'  Old gain   : {self.steering_to_servo_gain:.4f}  ->  '
              f'New gain : {new_gain:.4f}')
        print('-' * 70)

        if abs(new_gain) < GAIN_MIN_WARN or abs(new_gain) > GAIN_MAX_WARN:
            warn = input(f'  [warning] new gain magnitude {abs(new_gain):.3f} is outside '
                         f'[{GAIN_MIN_WARN}, {GAIN_MAX_WARN}]. Apply anyway? (y/n): ')
            if warn.strip().lower() != 'y':
                print('Not applied.\n')
                return True

        if input('Apply new gain? (y/n): ').strip().lower() != 'y':
            print('Kept current gain.\n')
            return True

        self.steering_to_servo_gain = new_gain
        self._write_gain(new_gain)
        print(SEP + '\n')
        return True

    def _read_xy(self, radius: float):
        """Return (x, y) floats, None to quit, or 'skip'. Validates range."""
        upper = radius * 3.0
        # x
        while self._active.is_set() and rclpy.ok():
            raw = input("Enter x_actual (m), or 'q' to quit, or 's' to skip: ").strip().lower()
            if raw == 'q':
                return None
            if raw == 's':
                return 'skip'
            try:
                x_actual = float(raw)
            except ValueError:
                print('  Not a number — try again.')
                continue
            if x_actual < 0 or x_actual > upper:
                print(f'  x out of range [0, {upper:.2f}] m — try again.')
                continue
            break
        else:
            return None
        # y (allow sign; range check on magnitude)
        while self._active.is_set() and rclpy.ok():
            raw = input('Enter y_actual (m): ').strip().lower()
            if raw == 'q':
                return None
            try:
                y_actual = float(raw)
            except ValueError:
                print('  Not a number — try again.')
                continue
            if abs(y_actual) < 0 or abs(y_actual) > upper:
                print(f'  y out of range [-{upper:.2f}, {upper:.2f}] m — try again.')
                continue
            break
        else:
            return None
        return (x_actual, y_actual)

    def _write_gain(self, new_gain: float):
        if self.vesc_yaml_path and os.path.isfile(self.vesc_yaml_path):
            if update_vesc_yaml(self.vesc_yaml_path, 'steering_angle_to_servo_gain', new_gain):
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
        print(f'Manually set  steering_angle_to_servo_gain: {new_gain:.4f}  '
              f'in vesc.yaml and restart.')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SteeringTuningNode()
    except ValueError:
        # invalid direction already logged
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
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
