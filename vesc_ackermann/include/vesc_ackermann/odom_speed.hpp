// -*- mode:c++; fill-column: 100; -*-
//
// odom_speed.hpp -- the ERPM -> speed conversion vesc_to_odom_backup integrates.
//
// Extracted as a pure function so the SIGN can be pinned by a unit test. The
// car reverses, and a reverse that reaches the wheels but not the odometry is
// invisible to everything downstream: /odom, the local EKF and mpc_corr's
// v_real all inherit whatever this returns.
//
// There is deliberately NO deadband here. vesc_driver already zeroes
// |rpm| < erpm_deadband on state.speed before publishing /sensors/core (see
// vesc_driver.cpp's own comment), so this node receives a signal that is
// already clean at standstill. It used to re-apply the same 500 ERPM gate and
// then a second 0.05 m/s one; both were unreachable (0.05 m/s is 226 ERPM, well
// inside the 500 the driver already removed) and are gone. Reinstating a gate
// here would silently widen the stack's dead zone in BOTH directions.

#ifndef VESC_ACKERMANN__ODOM_SPEED_HPP_
#define VESC_ACKERMANN__ODOM_SPEED_HPP_

namespace vesc_ackermann
{

// vesc.yaml's contract: erpm = speed_to_erpm_gain * speed + speed_to_erpm_offset,
// so speed = (erpm - offset) / gain. Linear and sign-transparent: a negative ERPM
// (the car rolling backwards) must come out a negative speed, never a magnitude.
inline double erpmToSpeed(double erpm, double speed_to_erpm_offset, double speed_to_erpm_gain)
{
  return (erpm - speed_to_erpm_offset) / speed_to_erpm_gain;
}

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ODOM_SPEED_HPP_
