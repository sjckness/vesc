// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-
//
// velocity_correction.hpp -- piecewise-linear low-speed velocity correction.
// Added by the VESC low-speed velocity correction pass, not part of the
// original vendored vesc_ackermann sources (see ackermann_to_vesc.hpp's own
// pre-existing @todo: "consider also providing an interpolated look-up
// table conversion" -- this is that).
//
// Problem this fixes: a single linear speed_to_erpm_gain is well-calibrated
// at 0.5 m/s (0% error, via vesc_tuning's speed_tuning_node) but
// speed_sweep_diagnostic_node's own sweep found serious nonlinearity below
// that -- 0.15 m/s commanded produced only 0.083 m/s actual (44% error),
// tapering to ~4-8% error by 0.35-0.4 m/s. Static friction/deadband
// dominates near zero; mechanical losses taper as speed increases -- there
// is no reason to expect this shape to be smooth, so this is deliberately
// PIECEWISE-LINEAR interpolation directly between measured points, not a
// fitted curve (a fitted curve could look plausible while being wrong
// between sparse knots, especially near the deadband where the true shape
// is least likely to be smooth).
//
// correctVelocity(desired_actual_mps, knots) is the pure function this file
// exists to provide -- given a desired ACTUAL velocity, interpolate the
// COMMANDED velocity (what to actually send into speed_to_erpm_gain) that
// the sweep data says would produce it. This mirrors, field-for-field,
// vesc_tuning's own Python velocity_correction.py (see that module's
// docstring for the offline/calibration-curve-building side of this same
// policy) -- kept as two independent implementations (this one, applied
// live per-command in ackermann_to_vesc_node; the Python one, for
// building/validating the calibration curve from a sweep CSV) rather than
// one shared library, since the two run in different languages/processes
// and the whole curve is only ever a handful of points -- not considered
// risky duplication at this size, and every policy branch here has a
// matching test in both languages' own test suites so they can't quietly
// drift apart unnoticed.
//
// Range policy (both ends deliberately NOT extrapolated into unmeasured
// territory -- identical policy, and identical wording, to velocity_
// correction.py's own module docstring):
//   - STRICTLY ABOVE the top knot's actual velocity: PASS THROUGH unchanged
//     (identity) -- the existing single linear speed_to_erpm_gain already
//     tracks well there (confirmed 0% error at the calibrated 0.5 m/s
//     point), so no correction is needed or wanted past it. AT the top
//     knot itself, the calibrated/interpolated range still applies (falls
//     through to ordinary interpolation, which reduces to exactly that
//     knot's own commanded value there) -- only genuine extrapolation
//     beyond a measured point is pass-through.
//   - AT OR BELOW the bottom knot's actual velocity: CLAMP to the bottom
//     knot's own commanded value (never a smaller commanded value than
//     what's known to actually move the car) -- this is exactly the
//     deadband/static-friction region, and extrapolating a smaller
//     commanded value risks silently commanding a speed that produces zero
//     real motion. An explicit clamp, not a silent linear guess.
//   - A literal zero (stop) request is bypassed entirely, unconditionally,
//     before any clamp/interpolation logic -- a stop command must always
//     reach the vehicle as a real zero, never distorted into "the smallest
//     calibrated commanded speed".
//   - No knots loaded at all: pass through unchanged, identical to this
//     feature not existing -- the default vesc.yaml state before a live
//     sweep has ever been run and its results pasted in.
//   - Symmetric in sign: negative (reverse) requests are corrected on their
//     magnitude and the sign is re-applied -- an assumption, not a measured
//     fact (the sweep only ever measured forward speeds), since there is no
//     reason to expect the deadband/friction shape to differ by direction.

#ifndef VESC_ACKERMANN__VELOCITY_CORRECTION_HPP_
#define VESC_ACKERMANN__VELOCITY_CORRECTION_HPP_

#include <cmath>
#include <utility>
#include <vector>

namespace vesc_ackermann
{

// One calibration point: (actual_mps, commanded_mps). A knot list passed to
// correctVelocity() must be sorted ascending by actual_mps -- this is the
// caller's responsibility (ackermann_to_vesc_node builds it once at startup
// directly from vesc.yaml's own sorted array keys; see that node's own
// comments) and is NOT re-validated here, to keep this function a plain,
// cheap, allocation-free per-command computation.
struct VelocityKnot
{
  double actual_mps;
  double commanded_mps;
};

inline double correctVelocity(double desired_actual_mps, const std::vector<VelocityKnot> & knots)
{
  // Explicit zero/stop bypass -- see file header's own policy note: never
  // distort an actual stop command through the deadband-floor clamp below.
  if (desired_actual_mps == 0.0) {
    return 0.0;
  }
  if (knots.empty()) {
    return desired_actual_mps;
  }

  const double sign = (desired_actual_mps < 0.0) ? -1.0 : 1.0;
  const double magnitude = std::fabs(desired_actual_mps);

  const double bottom_actual = knots.front().actual_mps;
  const double bottom_commanded = knots.front().commanded_mps;
  const double top_actual = knots.back().actual_mps;
  const double top_commanded = knots.back().commanded_mps;

  // Strictly ABOVE the top knot -- AT it is still part of the calibrated/
  // interpolated range (the loop below reduces to exactly the top knot's
  // own commanded value there) -- only genuine extrapolation past a
  // measured point passes through unchanged.
  if (magnitude > top_actual) {
    return sign * magnitude;
  }
  if (magnitude <= bottom_actual) {
    return sign * bottom_commanded;
  }

  for (size_t i = 1; i < knots.size(); ++i) {
    if (magnitude <= knots[i].actual_mps) {
      const VelocityKnot & lo = knots[i - 1];
      const VelocityKnot & hi = knots[i];
      const double t = (magnitude - lo.actual_mps) / (hi.actual_mps - lo.actual_mps);
      return sign * (lo.commanded_mps + t * (hi.commanded_mps - lo.commanded_mps));
    }
  }

  // Unreachable given the bounds checks above -- fail safe (the top knot's
  // own commanded value) rather than undefined behavior if it ever is.
  return sign * top_commanded;
}

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VELOCITY_CORRECTION_HPP_
