// Copyright 2026 f1tenth_more
//
// The ERPM -> speed conversion /odom is integrated from must carry the SIGN.
//
// Why this exists: on 2026-10-20 the car reversed three times (the BT logged
// stopping -> reverse, ending on timeout, bt_withdrawn and rear_clearance) while
// mpc_corr's v_real stayed at zero. Everything downstream of this function --
// /odom, the local EKF's /odometry/filtered, mpc_corr's v_real, and the reverse
// manoeuvre's own travelled distance -- inherits whatever it returns, so a
// magnitude here would make a reverse invisible to the whole stack.

#include <gtest/gtest.h>

#include <cmath>

#include "vesc_ackermann/odom_speed.hpp"

namespace
{
// vesc.yaml's shipped calibration.
constexpr double kGain = 4516.9;
constexpr double kOffset = 0.0;
}  // namespace

TEST(OdomSpeed, a_negative_erpm_becomes_a_negative_speed)
{
  // -0.4 m/s, the reverse manoeuvre's v_rev_mps, is -1806.76 ERPM.
  const double speed = vesc_ackermann::erpmToSpeed(-0.4 * kGain, kOffset, kGain);
  EXPECT_LT(speed, 0.0);
  EXPECT_NEAR(speed, -0.4, 1e-9);
}

TEST(OdomSpeed, forward_is_unchanged)
{
  const double speed = vesc_ackermann::erpmToSpeed(0.5 * kGain, kOffset, kGain);
  EXPECT_GT(speed, 0.0);
  EXPECT_NEAR(speed, 0.5, 1e-9);
}

TEST(OdomSpeed, the_conversion_is_odd_so_the_two_directions_cannot_drift_apart)
{
  for (const double erpm : {120.0, 375.0, 605.0, 1806.76, 2258.45}) {
    EXPECT_NEAR(
      vesc_ackermann::erpmToSpeed(-erpm, kOffset, kGain),
      -vesc_ackermann::erpmToSpeed(erpm, kOffset, kGain), 1e-12);
  }
}

TEST(OdomSpeed, no_deadband_is_applied_here)
{
  // 375 ERPM is 0.083 m/s: under vesc_driver's 500 gate, and under the 0.05 m/s
  // second gate this node used to apply. It must pass through untouched now --
  // the only deadband in the chain belongs to vesc_driver.
  const double slow = vesc_ackermann::erpmToSpeed(-375.0, kOffset, kGain);
  EXPECT_LT(slow, 0.0);
  EXPECT_NEAR(slow, -375.0 / kGain, 1e-12);
}

TEST(OdomSpeed, the_integrated_pose_moves_backwards)
{
  // vesc_to_odom_backup integrates x += speed * cos(yaw) * dt (yaw 0 = +x).
  const double speed = vesc_ackermann::erpmToSpeed(-0.4 * kGain, kOffset, kGain);
  const double dt = 0.1;
  double x = 0.0;
  for (int i = 0; i < 10; ++i) {
    x += speed * std::cos(0.0) * dt;
  }
  EXPECT_LT(x, 0.0);
  EXPECT_NEAR(x, -0.4, 1e-9);
}
