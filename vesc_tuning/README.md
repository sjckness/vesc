# vesc_tuning

Human-in-the-loop empirical calibration for the F1TENTH VESC stack:

1. **`speed_tuning_node`** — `speed_to_erpm_gain`: drive 1 m straight, measure
   the real distance.
2. **`steering_calibration_node`** — the 5 steering-calibration values
   (`servo_min`, `servo_max`, `steering_angle_to_servo_offset`,
   `steering_angle_to_servo_gain_left`, `steering_angle_to_servo_gain_right`):
   a menu-driven session covering range/bottleneck symmetry, offset, and
   left/right gain.
3. **`speed_sweep_diagnostic_node`** — read-only diagnostic (writes no config):
   sweeps a list of target speeds, driving each for a fixed duration and
   asking you to measure the actual distance travelled, to characterize how
   much commanded and actual speed diverge (mechanical losses) across the
   speed range. Prints a target-vs-actual-vs-error summary table and
   optionally a CSV. Useful to run **before**
   `sensor_covariance_calibration_node`'s `light_motion` mode, to sanity-check
   its `light_motion_target_speed` sits somewhere sane on this curve.

All three publish **directly** to the VESC driver topics (`/commands/motor/speed`
and `/commands/servo/position`, both `std_msgs/Float64`), bypassing
`ackermann_mux` and `ackermann_to_vesc`. Run them **without** the autonomous
stack (MPC) so nothing else fights for those topics.

> ⚠️ The VESC params are read **once at node startup** — there are no runtime
> parameter callbacks. After a value is written to disk you must **restart
> the vesc nodes** for it to take effect. Neither `speed_tuning_node`'s
> `vesc.yaml` nor `steering_calibration_node`'s `steering_calibration.yaml`
> is hot-reloaded, and `ros2 param set` will not help either.

> ℹ️ `speed_tuning_node` **seeds its starting gain from the current
> `vesc.yaml`** (when `vesc_yaml_path` is set), so each session continues
> from the last applied value instead of a fixed default; the declared
> `*_gain`/`*_offset` parameters are only fallbacks used when no yaml is
> available. `steering_calibration_node` instead treats
> `steering_calibration_yaml_path` as the one source of truth for its 5
> values — see its own section below.

## Recommended tuning order

1. **Speed gain first** (`speed_to_erpm_gain`, via `speed_tuning_node`).
   Steering Mode 1/3 drive at a target speed and use time to estimate arc
   length, so a correct speed gain makes those runs more accurate.
2. **Steering calibration second** (`steering_calibration_node`), in its own
   recommended order: Mode 1 (range/bottleneck) → Mode 2 (offset) →
   Mode 3 (left/right gain) — see below.

## Running

Use `ros2 run` for the interactive session (stdin is reliably connected):

```bash
# Speed gain
ros2 run vesc_tuning speed_tuning_node --ros-args \
  -p target_distance:=1.0 -p target_speed:=0.5 \
  -p vesc_yaml_path:=/path/to/f1tenth_bringup/config/vesc.yaml

# Steering calibration (menu-driven: range/offset/gain)
ros2 run vesc_tuning steering_calibration_node --ros-args \
  -p steering_calibration_yaml_path:=/path/to/f1tenth_hardware/config/steering_calibration.yaml

# Speed sweep diagnostic (read-only; writes no config)
ros2 run vesc_tuning speed_sweep_diagnostic_node --ros-args \
  -p target_speeds:="[0.15, 0.2, 0.25, 0.3, 0.35, 0.4]" -p drive_duration_sec:=3.0
```

A launch file (`speed_tuning.launch.py`) sets the same defaults for
`speed_tuning_node` and prints the tuning-mode warning. `steering_calibration_node`
and `speed_sweep_diagnostic_node` have **no launch file on purpose** — stdin is
not reliably forwarded to nodes started by `ros2 launch`, and both are entirely
stdin-driven, so always use `ros2 run` for them.

The vesc driver must be running so the commands reach the motor:
```bash
ros2 run vesc_driver vesc_driver_node --ros-args --params-file .../vesc.yaml \
  --params-file .../steering_calibration.yaml
```

## Speed procedure
1. Put the car on a clear straight ≥ 2 m.
2. Start the node, press ENTER. It drives `target_distance` at `target_speed`,
   then stops.
3. Measure the **actual** distance the car travelled (tape measure).
4. Type it in. The node prints old → new gain and asks to apply.
5. Apply → it writes `vesc.yaml`. **Restart** the vesc nodes, then repeat to
   verify the car now travels ~1 m.

## Steering calibration procedure

`steering_calibration_node` opens a text menu — `[1]` range/bottleneck
symmetry, `[2]` offset, `[3]` gain, `[q]` quit — and returns to it after each
mode finishes, so any step can be re-run without restarting the node. Every
confirmed change is saved immediately (atomically) to
`steering_calibration_yaml_path`, the same `steering_calibration.yaml` the
real `vesc_driver_node`/`ackermann_to_vesc_node` read — **restart those nodes**
to pick up each change.

1. **Mode 1 — range/bottleneck symmetry.** Drives full lock left, then full
   lock right, each a full circle; you measure and type each radius. The side
   with the *larger* radius steers less and is the mechanical bottleneck; the
   other side's `servo_min`/`servo_max` is pulled in to match it. Repeats
   until you confirm it's good.
2. **Mode 2 — offset.** Drives straight (servo centered at the current
   offset) for a fixed distance; you type the measured lateral deviation.
   `steering_angle_to_servo_offset` is corrected proportionally. Repeats
   until confirmed good.
3. **Mode 3 — gain.** Drives a circle sized to a target radius (default 1 m)
   for LEFT, then RIGHT, fully independently; you measure and type each
   radius, and that side's gain (`steering_angle_to_servo_gain_left` /
   `_right`) is rescaled by the measured/target ratio. Auto-continues while
   still outside tolerance, or asks you to confirm/retry, capped by
   `gain_max_iterations` per side.

See the `SteeringCalibrationNode` module docstring in
`vesc_tuning/steering_calibration_node.py` for the full derivation of each
mode's correction formula.

## Speed sweep diagnostic procedure

`speed_sweep_diagnostic_node` is read-only — it never writes `vesc.yaml` or
any other config. For each speed in `target_speeds` it: asks you to confirm,
drives at that constant commanded speed for `drive_duration_sec`, stops, asks
for the measured distance travelled, and immediately prints
target/actual/absolute-error/percent-error for that point. 'q' at any prompt
aborts the sweep early and still prints/writes whatever points were already
collected. After the last point (or an early abort), it prints a summary
table sorted by target speed and, unless `write_csv:=false`, writes the same
table to `speed_sweep_results/speed_sweep_<timestamp>.csv` in this package
(override the path with `csv_path:=...`).

Run it before `sensor_covariance_calibration_node`'s `light_motion` mode to
see where commanded and actual speed diverge across the range, so
`light_motion_target_speed` can be chosen somewhere the two still track each
other reasonably well.

## Gain-correction derivation (speed)
The motor command is `erpm = gain * speed + offset`. For a fixed drive time `t`
and target speed `v`, distance ∝ gain. With the old gain you measured
`measured_distance` instead of `target_distance`, so scale the gain by the ratio:

```
new_gain = old_gain * (target_distance / measured_distance)
```

If the car overshot (measured > target) the gain is reduced; if it fell short the
gain is increased.

## Safety
- A **stop** (zero speed + center servo) is published on startup, after every run,
  in `finally` blocks, and on Ctrl+C / `q`.
- Servo center/steering is always published **before** any motor command.
- Runs are hard-capped by `drive_duration_safety_max` seconds.
- Measurement inputs are range-checked; out-of-range gains require explicit
  confirmation before they are applied.

# Future: Automatic Tuning Node

This package is deliberately **human-in-the-loop**: the human is the position
sensor (tape measure) and the decision-maker (apply / reject). The architecture
is intentionally simple and dependency-free so it works on a bare car with no
external tracking.

A future `auto_tuning_node` could close the loop using on-board feedback instead
of a human:

- **Odometry feedback** — subscribe to `/odom` (VESC `vesc_to_odom`, or better,
  the ZED visual-inertial `/zed2/zed_node/odom`) to read the actual distance /
  final pose automatically, removing the manual measurement step.
- **Automatic iteration** — run several passes, apply the gain-correction
  formulas above each time, and stop when the error is within tolerance
  (e.g. < 2 %), implementing a simple proportional / bisection search on the gain.
- **Live application** — because the current VESC nodes have no runtime parameter
  callbacks, an auto-tuner would either (a) add a `set_parameters` callback to the
  conversion nodes, or (b) keep the write-YAML-and-restart approach but orchestrate
  the restart itself.
- **Keep the safety envelope** — the same stop-on-exit, duration caps and gain
  sanity bounds used here must remain, since an automatic loop drives the car
  unattended.
