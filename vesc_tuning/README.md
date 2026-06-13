# vesc_tuning

Human-in-the-loop empirical calibration of the two most important F1TENTH VESC
gains:

1. **`speed_to_erpm_gain`** — drive 1 m straight, measure the real distance.
2. **`steering_angle_to_servo_gain`** — drive a quarter circle, measure where you end up.

Both nodes follow the same loop:

> command → car moves → **you** measure → **you** type the result → node recomputes
> the gain → writes `vesc.yaml` → prompts you to restart the vesc nodes → repeat.

The nodes publish **directly** to the VESC driver topics
(`/commands/motor/speed` and `/commands/servo/position`, both `std_msgs/Float64`),
bypassing `ackermann_mux` and `ackermann_to_vesc`. Run them **without** the
autonomous stack (MPC) so nothing else fights for those topics.

> ⚠️ The VESC params are read **once at node startup** — there are no runtime
> parameter callbacks. After a gain is written to `vesc.yaml` you must **restart
> the vesc nodes** for it to take effect.

## Recommended tuning order

1. **Speed gain first** (`speed_to_erpm_gain`). The steering test drives at a
   target speed and uses time to estimate arc length, so a correct speed gain
   makes the steering run more accurate.
2. **Steering gain second** (`steering_angle_to_servo_gain`).

## Running

Use `ros2 run` for the interactive session (stdin is reliably connected):

```bash
# Speed gain
ros2 run vesc_tuning speed_tuning_node --ros-args \
  -p target_distance:=1.0 -p target_speed:=0.5 \
  -p vesc_yaml_path:=/path/to/f1tenth_stack/config/vesc.yaml

# Steering gain
ros2 run vesc_tuning steering_tuning_node --ros-args \
  -p steering_angle:=0.26 -p direction:=left -p target_speed:=0.5 \
  -p vesc_yaml_path:=/path/to/f1tenth_stack/config/vesc.yaml
```

Launch files (`speed_tuning.launch.py`, `steering_tuning.launch.py`) set the same
defaults and default `vesc_yaml_path` to the installed `f1tenth_stack/config/vesc.yaml`,
and print the tuning-mode warning. Note: stdin is **not** reliably forwarded to
nodes started by `ros2 launch`, so prefer `ros2 run` for the interactive part.

The vesc driver must be running so the commands reach the motor:
```bash
ros2 run vesc_driver vesc_driver_node --ros-args --params-file .../vesc.yaml
```

## Speed procedure
1. Put the car on a clear straight ≥ 2 m.
2. Start the node, press ENTER. It drives `target_distance` at `target_speed`,
   then stops.
3. Measure the **actual** distance the car travelled (tape measure).
4. Type it in. The node prints old → new gain and asks to apply.
5. Apply → it writes `vesc.yaml`. **Restart** the vesc nodes, then repeat to
   verify the car now travels ~1 m.

## Steering procedure
1. Mark the start point and the car's heading (+X).
2. Start the node, press ENTER. It drives a quarter circle (default left, 0.26 rad).
3. Measure the **actual** final position `(x, y)` relative to the start, in metres
   (left turn ends near `(+R, +R)`, right turn near `(+R, −R)`).
4. Type `x` then `y`. The node prints expected vs actual radius and old → new gain.
5. Apply → writes `vesc.yaml`. **Restart** vesc nodes and repeat to verify.

## Gain-correction derivations (plain text)

### Speed gain
The motor command is `erpm = gain * speed + offset`. For a fixed drive time `t`
and target speed `v`, distance ∝ gain. With the old gain you measured
`measured_distance` instead of `target_distance`, so scale the gain by the ratio:

```
new_gain = old_gain * (target_distance / measured_distance)
```

If the car overshot (measured > target) the gain is reduced; if it fell short the
gain is increased.

### Steering gain
Bicycle model radius for a commanded steering angle `delta`:

```
R_expected = wheelbase / tan(delta)
```

A perfect 90° arc from origin (heading +X) ends at distance `R*sqrt(2)` from the
start (the chord of a quarter circle of radius R). So from the measured final
point:

```
R_actual = sqrt(x_actual^2 + y_actual^2) / sqrt(2)
```

The servo command is `servo = gain * delta + offset`. If the real radius differs
from expected, the effective steering differed from the commanded `delta`; scale
the gain by the radius ratio:

```
new_gain = old_gain * (R_actual / R_expected)
error_pct = (R_actual - R_expected) / R_expected * 100
```

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
