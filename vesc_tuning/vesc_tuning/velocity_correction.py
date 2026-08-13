"""Piecewise-linear low-speed velocity correction, built from
speed_sweep_diagnostic_node.py's own commanded-vs-actual CSV output.

Problem this fixes: a single linear speed_to_erpm_gain is well-calibrated
at 0.5 m/s (0% error, via speed_tuning_node.py's own separate calibration)
but the sweep found serious nonlinearity below that -- 0.15 m/s commanded
produced only 0.083 m/s actual (44% error), tapering down to ~4-8% error
by 0.35-0.4 m/s. Static friction/deadband dominates near zero; mechanical
losses taper as speed increases -- there's no reason to expect this shape
to be smooth, so this is deliberately a PIECEWISE-LINEAR interpolation
directly between measured points, not a fitted curve (a fitted curve could
look plausible while being wrong between sparse knots, especially near the
deadband where the true shape is least likely to be smooth).

correct_velocity(desired_actual_mps, knots) is the pure function this
module exists to provide -- given a desired ACTUAL velocity, interpolate
the COMMANDED velocity (what to actually send into speed_to_erpm_gain)
that the sweep data says would produce it. This mirrors, field-for-field,
vesc_ackermann's own C++ velocity_correction.hpp (see that file for the
runtime/live-node side of this same policy) -- kept as two independent
implementations (Python here, for building/validating the calibration
curve from a CSV; C++ there, for the live per-command correction in
ackermann_to_vesc_node) rather than one shared library, since the two
run in different languages/processes and the whole curve is only ever a
handful of points -- see that file's own docstring for why this is not
considered risky duplication.

Range policy (both ends deliberately NOT extrapolated into unmeasured
territory):
  - STRICTLY ABOVE the top knot's actual velocity: PASS THROUGH unchanged
    (identity) -- the existing single linear speed_to_erpm_gain already
    tracks well there (confirmed 0% error at the calibrated 0.5 m/s
    point), so no correction is needed or wanted past it. AT the top knot
    itself, the calibrated/interpolated range still applies (falls through
    to ordinary interpolation, which reduces to exactly that knot's own
    commanded value there) -- only genuine extrapolation beyond a measured
    point is pass-through.
  - AT OR BELOW the bottom knot's actual velocity: CLAMP to the bottom
    knot's own commanded value (never a smaller commanded value than
    what's known to actually move the car) -- this is exactly the
    deadband/static-friction region, and extrapolating a smaller
    commanded value risks silently commanding a speed that produces zero
    real motion. An explicit clamp, not a silent linear guess.
  - A literal zero (stop) request is bypassed entirely, unconditionally,
    before any clamp/interpolation logic -- a stop command must always
    reach the vehicle as a real zero, never distorted into "the smallest
    calibrated commanded speed".
  - No knots loaded at all: pass through unchanged, identical to this
    feature not existing (matches the same "no calibration data yet ==
    no-op, not a guess" reasoning threaded through this whole codebase's
    other calibration-consuming nodes).
"""

import csv
from pathlib import Path
from typing import List, Sequence, Tuple

Knot = Tuple[float, float]  # (actual_mps, commanded_mps), sorted by actual_mps ascending


def load_knots_from_csv(csv_path) -> List[Knot]:
    """Read a speed_sweep_diagnostic_node.py results CSV (columns
    target_speed_mps, actual_speed_mps, abs_error_mps, pct_error -- see
    that node's own _write_csv) and return (actual_mps, commanded_mps)
    knot pairs, sorted ascending by actual_mps -- the CSV's own rows
    already are the calibration knots directly; no fitting/smoothing
    happens here, just extraction + the axis swap correct_velocity's
    interpolation needs (actual is the independent variable, since the
    whole point is "given a desired ACTUAL speed, what COMMANDED speed
    produces it").
    """
    knots: List[Knot] = []
    with open(Path(csv_path), 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            commanded = float(row['target_speed_mps'])
            actual = float(row['actual_speed_mps'])
            knots.append((actual, commanded))
    knots.sort(key=lambda k: k[0])
    return knots


def correct_velocity(desired_actual_mps: float, knots: Sequence[Knot]) -> float:
    """Piecewise-linear inverse interpolation: given a desired ACTUAL
    velocity (m/s) and a sorted-by-actual_mps knot list (see
    load_knots_from_csv), return the corrected COMMANDED velocity to feed
    into speed_to_erpm_gain. See module docstring for the full range
    policy (pass-through above the top knot, clamp below the bottom knot,
    zero always bypasses correction, empty knots == pass-through).

    Symmetric in sign: negative (reverse) requests are corrected on their
    magnitude and the sign is re-applied -- an assumption, not a measured
    fact (the sweep only ever measured forward speeds), since there is no
    reason to expect the deadband/friction shape to differ by direction
    and leaving reverse uncorrected would be a stranger default than
    extending the same curve to it.
    """
    if desired_actual_mps == 0.0:
        return 0.0
    if not knots:
        return desired_actual_mps

    sign = -1.0 if desired_actual_mps < 0.0 else 1.0
    magnitude = abs(desired_actual_mps)

    bottom_actual, bottom_commanded = knots[0]
    top_actual, top_commanded = knots[-1]

    # Strictly ABOVE the top knot -- AT it is still part of the calibrated/
    # interpolated range (falls through to the loop below, which reduces
    # to exactly the top knot's own commanded value there) -- only genuine
    # extrapolation past a measured point passes through unchanged.
    if magnitude > top_actual:
        return sign * magnitude
    if magnitude <= bottom_actual:
        return sign * bottom_commanded

    for i in range(1, len(knots)):
        hi_actual, hi_commanded = knots[i]
        if magnitude <= hi_actual:
            lo_actual, lo_commanded = knots[i - 1]
            t = (magnitude - lo_actual) / (hi_actual - lo_actual)
            return sign * (lo_commanded + t * (hi_commanded - lo_commanded))

    # Unreachable given the bounds checks above (top_actual check already
    # handles magnitude >= knots[-1][0]) -- fail safe rather than raise.
    return sign * top_commanded


def format_yaml_knots(knots: Sequence[Knot]) -> str:
    """Render `knots` as ready-to-paste vesc.yaml lines (the two parallel
    array keys ackermann_to_vesc_node reads -- see that file's own vesc.yaml
    comments). A convenience for turning a fresh sweep CSV into the actual
    config after a live calibration run -- deliberately NOT an automatic
    writer (vesc.yaml is a large, hand-maintained, comment-heavy shared
    file; a targeted array-aware in-place editor is a bigger, separate
    piece of work than this task's own scope, and pasting two short array
    lines by hand is a small, low-risk step for a human who just finished
    driving the sweep anyway). Remember: this workspace is not
    --symlink-install -- edits to src/f1tenth_bringup/config/vesc.yaml need
    a colcon build before ackermann_to_vesc_node's next (re)start will see
    them (see f1tenth_hardware/launch/vesc.launch.py's own note on this).
    """
    actual_vals = ', '.join(f'{a:.6f}' for a, _ in knots)
    commanded_vals = ', '.join(f'{c:.6f}' for _, c in knots)
    return (
        f'    velocity_correction_actual_mps: [{actual_vals}]\n'
        f'    velocity_correction_commanded_mps: [{commanded_vals}]\n'
    )


def main():
    """CLI convenience: `python3 -m vesc_tuning.velocity_correction <csv_path>`
    prints the ready-to-paste vesc.yaml lines for that sweep's results."""
    import sys
    if len(sys.argv) != 2:
        print('usage: python3 -m vesc_tuning.velocity_correction <sweep_csv_path>')
        sys.exit(1)
    knots = load_knots_from_csv(sys.argv[1])
    print(f'# {len(knots)} knot(s) loaded from {sys.argv[1]}')
    print(format_yaml_knots(knots))


if __name__ == '__main__':
    main()
