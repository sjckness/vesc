"""First real logic tests in vesc_tuning (previously only the stock ament
linter boilerplate). Added by the VESC low-speed velocity correction pass.

Mirrors, scenario-for-scenario, vesc_ackermann's own C++
test/test_velocity_correction.cpp -- both implementations (this Python one,
which builds/validates the calibration curve from a sweep CSV; the C++ one,
which applies it live in ackermann_to_vesc_node) are meant to agree on
every policy branch, so the same cases are exercised in both languages
rather than trusting one to imply the other. See velocity_correction.py's
own module docstring for the full range-policy reasoning.

Run standalone: python3 -m pytest test/test_velocity_correction.py -v
"""

import csv

import pytest

from vesc_tuning.velocity_correction import (
    correct_velocity,
    format_yaml_knots,
    load_knots_from_csv,
)

# Small, hand-computable knot set -- (actual_mps, commanded_mps), matching
# the shape (worse correction needed at low speed, ~identity by 0.50) the
# real sweep data found, but with round numbers a test can check by hand.
KNOTS = [
    (0.05, 0.10),
    (0.10, 0.14),
    (0.15, 0.18),
    (0.20, 0.22),
    (0.50, 0.50),
]

SPEED_TO_ERPM_GAIN = 5499.271647286143
SPEED_TO_ERPM_OFFSET = 0.0


class TestCorrectVelocity:

    def test_exact_knot_returns_its_own_commanded_value(self):
        assert correct_velocity(0.15, KNOTS) == pytest.approx(0.18)
        assert correct_velocity(0.05, KNOTS) == pytest.approx(0.10)
        assert correct_velocity(0.50, KNOTS) == pytest.approx(0.50)

    def test_midpoint_between_two_knots_interpolates_linearly(self):
        # Midpoint of (0.10, 0.14) and (0.15, 0.18) -- desired=0.125 is
        # exactly halfway, so the corrected command should be exactly
        # halfway too: (0.14 + 0.18) / 2 = 0.16.
        assert correct_velocity(0.125, KNOTS) == pytest.approx(0.16)

    def test_quarter_point_between_two_knots(self):
        # 1/4 of the way from (0.15, 0.18) to (0.20, 0.22): 0.18 + 0.25*0.04
        assert correct_velocity(0.1625, KNOTS) == pytest.approx(0.19)

    def test_above_top_knot_passes_through_unchanged(self):
        # NOT clamped to the top knot's commanded value (0.50) -- passed
        # through as-is, since the existing linear gain already tracks
        # well above the calibrated range.
        assert correct_velocity(0.7, KNOTS) == pytest.approx(0.7)
        assert correct_velocity(1.5, KNOTS) == pytest.approx(1.5)

    def test_at_top_knot_exactly_uses_the_calibrated_value_not_passthrough(self):
        # Regression test for a real bug found while writing this suite:
        # KNOTS' own top point (0.50, 0.50) can't distinguish "pass
        # through" from "use the knot's value" since they coincide here --
        # use a knot set where the top point's commanded value DIFFERS
        # from its actual value, so AT the boundary the two policies give
        # different answers, and confirm the calibrated one wins (only
        # STRICTLY ABOVE the top knot should pass through unchanged).
        knots_with_correction_at_top = [(0.10, 0.14), (0.50, 0.47)]
        assert correct_velocity(0.50, knots_with_correction_at_top) == pytest.approx(0.47)
        # Just above it, pass-through takes over.
        assert correct_velocity(0.51, knots_with_correction_at_top) == pytest.approx(0.51)

    def test_below_bottom_knot_clamps_not_extrapolates(self):
        # Must NOT linearly extrapolate toward zero (which could guess a
        # smaller commanded value than what's known to actually move the
        # car) -- clamps to the bottom knot's own commanded value.
        assert correct_velocity(0.02, KNOTS) == pytest.approx(0.10)
        assert correct_velocity(0.001, KNOTS) == pytest.approx(0.10)

    def test_zero_bypasses_correction_entirely(self):
        # A real stop command must reach the vehicle as a real zero, never
        # distorted into "the smallest calibrated commanded speed" (0.10
        # here) via the below-bottom clamp.
        assert correct_velocity(0.0, KNOTS) == 0.0

    def test_negative_speed_mirrors_the_positive_curve(self):
        assert correct_velocity(-0.15, KNOTS) == pytest.approx(-0.18)
        assert correct_velocity(-0.125, KNOTS) == pytest.approx(-0.16)
        assert correct_velocity(-0.02, KNOTS) == pytest.approx(-0.10)
        assert correct_velocity(-0.7, KNOTS) == pytest.approx(-0.7)

    def test_empty_knots_passes_through_unchanged(self):
        # No calibration data loaded yet -- identical to this feature not
        # existing, not a guess.
        assert correct_velocity(0.3, []) == pytest.approx(0.3)
        assert correct_velocity(0.0, []) == 0.0

    def test_single_knot_acts_as_both_top_and_bottom(self):
        single = [(0.2, 0.25)]
        assert correct_velocity(0.2, single) == pytest.approx(0.25)
        assert correct_velocity(0.1, single) == pytest.approx(0.25)  # clamp
        assert correct_velocity(0.5, single) == pytest.approx(0.5)   # pass-through


class TestComposesWithSpeedToErpmGain:
    """Confirms the corrected command still flows through the SAME
    downstream ERPM conversion unchanged -- this stage corrects the INPUT
    to speed_to_erpm_gain, it does not replace it."""

    def test_erpm_uses_corrected_command_not_raw_desired(self):
        desired = 0.125  # would produce a different erpm if uncorrected
        corrected = correct_velocity(desired, KNOTS)
        erpm = SPEED_TO_ERPM_GAIN * corrected + SPEED_TO_ERPM_OFFSET
        assert corrected == pytest.approx(0.16)
        assert erpm == pytest.approx(SPEED_TO_ERPM_GAIN * 0.16)
        assert erpm != pytest.approx(SPEED_TO_ERPM_GAIN * desired)

    def test_erpm_for_a_stop_command_is_exactly_zero(self):
        erpm = SPEED_TO_ERPM_GAIN * correct_velocity(0.0, KNOTS) + SPEED_TO_ERPM_OFFSET
        assert erpm == 0.0

    def test_erpm_above_top_knot_matches_uncorrected_linear_behavior(self):
        desired = 1.0
        corrected = correct_velocity(desired, KNOTS)
        erpm_corrected = SPEED_TO_ERPM_GAIN * corrected + SPEED_TO_ERPM_OFFSET
        erpm_uncorrected = SPEED_TO_ERPM_GAIN * desired + SPEED_TO_ERPM_OFFSET
        assert erpm_corrected == pytest.approx(erpm_uncorrected)


class TestLoadKnotsFromCsv:

    def test_round_trips_a_sweep_csv(self, tmp_path):
        csv_path = tmp_path / 'sweep.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['target_speed_mps', 'actual_speed_mps', 'abs_error_mps', 'pct_error'])
            # Written out of order deliberately -- load must sort by actual.
            writer.writerow(['0.500000', '0.500000', '0.000000', '0.000000'])
            writer.writerow(['0.150000', '0.083000', '0.067000', '44.666667'])
            writer.writerow(['0.300000', '0.277000', '0.023000', '7.666667'])

        knots = load_knots_from_csv(csv_path)
        assert knots == [
            pytest.approx((0.083, 0.15)),
            pytest.approx((0.277, 0.30)),
            pytest.approx((0.500, 0.50)),
        ]

    def test_accepts_a_plain_string_path(self, tmp_path):
        csv_path = tmp_path / 'sweep.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['target_speed_mps', 'actual_speed_mps', 'abs_error_mps', 'pct_error'])
            writer.writerow(['0.200000', '0.157000', '0.043000', '21.5'])
        knots = load_knots_from_csv(str(csv_path))
        assert knots == [pytest.approx((0.157, 0.20))]


class TestFormatYamlKnots:

    def test_produces_two_parallel_array_lines(self):
        text = format_yaml_knots(KNOTS)
        assert 'velocity_correction_actual_mps: [' in text
        assert 'velocity_correction_commanded_mps: [' in text
        lines = text.strip().splitlines()
        assert len(lines) == 2
        # Same number of comma-separated values on both lines.
        actual_count = lines[0].count(',') + 1
        commanded_count = lines[1].count(',') + 1
        assert actual_count == commanded_count == len(KNOTS)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
