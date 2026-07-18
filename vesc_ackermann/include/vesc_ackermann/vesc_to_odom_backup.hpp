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

// NOTE: This is an unmodified backup copy of the original bicycle-model
// vesc_to_odom node, preserved as the executable `vesc_to_odom_node_backup`.
// The class and node name are renamed (VescToOdomBackup / vesc_to_odom_node_backup)
// only so it can coexist with the new fused node; the odometry logic is identical.
#ifndef VESC_ACKERMANN__VESC_TO_ODOM_BACKUP_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_BACKUP_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using nav_msgs::msg::Odometry;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class VescToOdomBackup : public rclcpp::Node
{
public:
  explicit VescToOdomBackup(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  /** State message does not report servo position, so use the command instead */
  bool use_servo_cmd_;
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double wheelbase_;
  bool publish_tf_;
  /** Symmetric deadband [ERPM] applied to raw state.speed before gain conversion, to
   *  reject hall-sensor standstill noise (observed phantom 346-384 ERPM at true rest). */
  double erpm_deadband_;
  /** Message-level (Level 1) covariance calibration: measured stationary variance of
   *  twist.twist.linear.x, applied to twist.covariance[0]. vy has no equivalent -- it's
   *  a hardcoded 0.0 constant (Ackermann kinematics), not a measured quantity, so there
   *  is no vy_variance. Default 0.0 = uncalibrated. See f1tenth_diagnostics'
   *  sensor_covariance_calibration_node. */
  double vx_variance_;

  // odometry state
  double x_, y_, yaw_;
  /** Last servo position commanded value (/commands/servo/position). Null until the
   *  first message arrives -- vescStateCallback treats null as steering angle 0.0
   *  (straight-ahead) rather than deferring publication, so /odom starts flowing as
   *  soon as sensors/core does, even before any drive command has ever been sent. */
  Float64::SharedPtr last_servo_cmd_;
  VescStateStamped::SharedPtr last_state_;  ///< Last received state message

  // ROS services
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<Float64>::SharedPtr servo_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // ROS callbacks
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void servoCmdCallback(const Float64::SharedPtr servo);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_BACKUP_HPP_
