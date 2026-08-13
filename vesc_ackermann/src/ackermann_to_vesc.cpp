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

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_left_ =
    declare_parameter<double>("steering_angle_to_servo_gain_left");
  steering_to_servo_gain_right_ =
    declare_parameter<double>("steering_angle_to_servo_gain_right");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");

  // Piecewise-linear low-speed velocity correction (see velocity_correction.hpp).
  // Two parallel arrays, not a single flattened one, so vesc.yaml's own array
  // syntax stays a plain flat list of numbers per key -- easier to read/hand-edit
  // than an interleaved or nested structure. Empty-vector defaults: a vesc.yaml
  // that hasn't had a sweep's results pasted in yet still starts up fine, with
  // correctVelocity() falling back to pure pass-through (see that function's own
  // policy doc) -- identical to this feature not existing, not a crash.
  // NOTE: the default must be an explicit std::vector<double>{}, not a bare {} --
  // bare {} is ambiguous with declare_parameter's 3-argument (name, default,
  // ParameterDescriptor) overload and throws
  // UninitializedStaticallyTypedParameterException at runtime instead of quietly
  // defaulting to empty (confirmed live via a standalone rclcpp test program before
  // trusting this pattern here -- do not "simplify" this back to {}).
  const auto velocity_correction_actual_mps =
    declare_parameter<std::vector<double>>("velocity_correction_actual_mps", std::vector<double>{});
  const auto velocity_correction_commanded_mps =
    declare_parameter<std::vector<double>>(
    "velocity_correction_commanded_mps", std::vector<double>{});
  if (velocity_correction_actual_mps.size() != velocity_correction_commanded_mps.size()) {
    RCLCPP_ERROR(
      get_logger(),
      "velocity_correction_actual_mps (%zu entries) and "
      "velocity_correction_commanded_mps (%zu entries) have mismatched lengths -- "
      "ignoring both, no velocity correction will be applied. Check vesc.yaml.",
      velocity_correction_actual_mps.size(), velocity_correction_commanded_mps.size());
  } else {
    velocity_correction_knots_.reserve(velocity_correction_actual_mps.size());
    for (size_t i = 0; i < velocity_correction_actual_mps.size(); ++i) {
      velocity_correction_knots_.push_back(
        {velocity_correction_actual_mps[i], velocity_correction_commanded_mps[i]});
    }
    // correctVelocity() requires (and does not itself re-check, see its own
    // docs) that knots are sorted ascending by actual_mps -- a plain
    // sequential scan here at startup, once, to catch a hand-edit mistake
    // loudly instead of silently interpolating against a nonsensical curve.
    for (size_t i = 1; i < velocity_correction_knots_.size(); ++i) {
      if (velocity_correction_knots_[i].actual_mps <=
        velocity_correction_knots_[i - 1].actual_mps)
      {
        RCLCPP_ERROR(
          get_logger(),
          "velocity_correction_actual_mps is not strictly ascending (entry %zu: %.4f "
          "<= entry %zu: %.4f) -- clearing all velocity correction knots, no "
          "correction will be applied. Fix vesc.yaml's ordering and restart.",
          i, velocity_correction_knots_[i].actual_mps,
          i - 1, velocity_correction_knots_[i - 1].actual_mps);
        velocity_correction_knots_.clear();
        break;
      }
    }
    RCLCPP_INFO(
      get_logger(), "Velocity correction: %zu knot(s) loaded.",
      velocity_correction_knots_.size());
  }

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // Piecewise-linear low-speed velocity correction (see velocity_correction.hpp) --
  // corrects the DESIRED speed into a CORRECTED commanded speed BEFORE the existing
  // linear speed_to_erpm_gain conversion below, i.e. it corrects that stage's own
  // input rather than replacing it. A no-op (returns cmd->drive.speed unchanged)
  // when velocity_correction_knots_ is empty (no sweep calibrated yet).
  const double corrected_speed = correctVelocity(cmd->drive.speed, velocity_correction_knots_);

  // calc vesc electric RPM (speed)
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * corrected_speed + speed_to_erpm_offset_;

  // calc steering angle (servo) -- asymmetric linkage: left (angle >= 0)
  // and right (angle < 0) use separately-calibrated gains.
  Float64 servo_msg;
  const double steering_to_servo_gain =
    (cmd->drive.steering_angle >= 0.0) ? steering_to_servo_gain_left_ : steering_to_servo_gain_right_;
  servo_msg.data = steering_to_servo_gain * cmd->drive.steering_angle + steering_to_servo_offset_;

  // publish
  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
