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
// First real logic tests in vesc_ackermann. Added by the VESC low-speed
// velocity correction pass.
//
// Mirrors, scenario-for-scenario, vesc_tuning's own Python
// test/test_velocity_correction.py -- both implementations are meant to
// agree on every policy branch, so the same cases are exercised in both
// languages rather than trusting one to imply the other. See
// velocity_correction.hpp's own file header for the full range-policy
// reasoning.

#include <gtest/gtest.h>

#include <vector>

#include "vesc_ackermann/velocity_correction.hpp"

namespace
{

using vesc_ackermann::correctVelocity;
using vesc_ackermann::VelocityKnot;

// Small, hand-computable knot set -- (actual_mps, commanded_mps), matching
// the shape (worse correction needed at low speed, ~identity by 0.50) the
// real sweep data found, but with round numbers a test can check by hand.
// Identical values to vesc_tuning's own Python test suite's KNOTS.
const std::vector<VelocityKnot> kKnots = {
  {0.05, 0.10},
  {0.10, 0.14},
  {0.15, 0.18},
  {0.20, 0.22},
  {0.50, 0.50},
};

constexpr double kSpeedToErpmGain = 5499.271647286143;
constexpr double kSpeedToErpmOffset = 0.0;
constexpr double kTol = 1e-9;

}  // namespace

TEST(VelocityCorrection, ExactKnotReturnsItsOwnCommandedValue) {
  EXPECT_NEAR(correctVelocity(0.15, kKnots), 0.18, kTol);
  EXPECT_NEAR(correctVelocity(0.05, kKnots), 0.10, kTol);
  EXPECT_NEAR(correctVelocity(0.50, kKnots), 0.50, kTol);
}

TEST(VelocityCorrection, MidpointBetweenTwoKnotsInterpolatesLinearly) {
  // Midpoint of (0.10, 0.14) and (0.15, 0.18): (0.14 + 0.18) / 2 = 0.16.
  EXPECT_NEAR(correctVelocity(0.125, kKnots), 0.16, kTol);
}

TEST(VelocityCorrection, QuarterPointBetweenTwoKnots) {
  // 1/4 of the way from (0.15, 0.18) to (0.20, 0.22): 0.18 + 0.25*0.04.
  EXPECT_NEAR(correctVelocity(0.1625, kKnots), 0.19, kTol);
}

TEST(VelocityCorrection, AboveTopKnotPassesThroughUnchanged) {
  EXPECT_NEAR(correctVelocity(0.7, kKnots), 0.7, kTol);
  EXPECT_NEAR(correctVelocity(1.5, kKnots), 1.5, kTol);
}

TEST(VelocityCorrection, AtTopKnotExactlyUsesTheCalibratedValueNotPassthrough) {
  // Regression case: a knot set where the top point's commanded value
  // DIFFERS from its actual value, so AT the boundary the two policies
  // (pass-through vs. use-the-knot) give different answers -- confirms the
  // calibrated one wins (only STRICTLY ABOVE the top knot passes through).
  const std::vector<VelocityKnot> knots_with_correction_at_top = {
    {0.10, 0.14},
    {0.50, 0.47},
  };
  EXPECT_NEAR(correctVelocity(0.50, knots_with_correction_at_top), 0.47, kTol);
  EXPECT_NEAR(correctVelocity(0.51, knots_with_correction_at_top), 0.51, kTol);
}

TEST(VelocityCorrection, BelowBottomKnotClampsNotExtrapolates) {
  // Must NOT linearly extrapolate toward zero -- clamps to the bottom
  // knot's own commanded value.
  EXPECT_NEAR(correctVelocity(0.02, kKnots), 0.10, kTol);
  EXPECT_NEAR(correctVelocity(0.001, kKnots), 0.10, kTol);
}

TEST(VelocityCorrection, ZeroBypassesCorrectionEntirely) {
  // A real stop command must reach the vehicle as a real zero, never
  // distorted into "the smallest calibrated commanded speed" (0.10 here)
  // via the below-bottom clamp.
  EXPECT_EQ(correctVelocity(0.0, kKnots), 0.0);
}

TEST(VelocityCorrection, NegativeSpeedMirrorsThePositiveCurve) {
  EXPECT_NEAR(correctVelocity(-0.15, kKnots), -0.18, kTol);
  EXPECT_NEAR(correctVelocity(-0.125, kKnots), -0.16, kTol);
  EXPECT_NEAR(correctVelocity(-0.02, kKnots), -0.10, kTol);
  EXPECT_NEAR(correctVelocity(-0.7, kKnots), -0.7, kTol);
}

TEST(VelocityCorrection, EmptyKnotsPassesThroughUnchanged) {
  const std::vector<VelocityKnot> empty;
  EXPECT_NEAR(correctVelocity(0.3, empty), 0.3, kTol);
  EXPECT_EQ(correctVelocity(0.0, empty), 0.0);
}

TEST(VelocityCorrection, SingleKnotActsAsBothTopAndBottom) {
  const std::vector<VelocityKnot> single = {{0.2, 0.25}};
  EXPECT_NEAR(correctVelocity(0.2, single), 0.25, kTol);
  EXPECT_NEAR(correctVelocity(0.1, single), 0.25, kTol);  // clamp
  EXPECT_NEAR(correctVelocity(0.5, single), 0.5, kTol);   // pass-through
}

// ---- Composition with the existing speed_to_erpm_gain stage ---------------
// Confirms the corrected command still flows through the SAME downstream
// ERPM conversion unchanged -- this stage corrects the INPUT to
// speed_to_erpm_gain, it does not replace it. Mirrors
// ackermannCmdCallback's own two-line sequence (correct, then gain*x+offset).

TEST(VelocityCorrectionComposesWithErpmGain, UsesCorrectedCommandNotRawDesired) {
  const double desired = 0.125;  // would produce a different erpm if uncorrected
  const double corrected = correctVelocity(desired, kKnots);
  const double erpm = kSpeedToErpmGain * corrected + kSpeedToErpmOffset;
  EXPECT_NEAR(corrected, 0.16, kTol);
  EXPECT_NEAR(erpm, kSpeedToErpmGain * 0.16, 1e-6);
  EXPECT_NE(erpm, kSpeedToErpmGain * desired + kSpeedToErpmOffset);
}

TEST(VelocityCorrectionComposesWithErpmGain, StopCommandProducesExactlyZeroErpm) {
  const double erpm = kSpeedToErpmGain * correctVelocity(0.0, kKnots) + kSpeedToErpmOffset;
  EXPECT_EQ(erpm, 0.0);
}

TEST(VelocityCorrectionComposesWithErpmGain, AboveTopKnotMatchesUncorrectedLinearBehavior) {
  const double desired = 1.0;
  const double corrected = correctVelocity(desired, kKnots);
  const double erpm_corrected = kSpeedToErpmGain * corrected + kSpeedToErpmOffset;
  const double erpm_uncorrected = kSpeedToErpmGain * desired + kSpeedToErpmOffset;
  EXPECT_NEAR(erpm_corrected, erpm_uncorrected, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
