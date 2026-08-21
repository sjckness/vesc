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

#ifndef VESC_DRIVER__VESC_DRIVER_HPP_
#define VESC_DRIVER__VESC_DRIVER_HPP_

#include <experimental/optional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <vesc_msgs/msg/vesc_imu.hpp>
#include <vesc_msgs/msg/vesc_imu_stamped.hpp>

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet.hpp"

namespace vesc_driver
{

using std_msgs::msg::Float64;
using vesc_msgs::msg::VescState;
using vesc_msgs::msg::VescStateStamped;
using vesc_msgs::msg::VescImuStamped;
using sensor_msgs::msg::Imu;

class VescDriver
  : public rclcpp::Node
{
public:
  explicit VescDriver(const rclcpp::NodeOptions & options);

private:
  // interface to the VESC
  VescInterface vesc_;
  void vescPacketCallback(const std::shared_ptr<VescPacket const> & packet);
  void vescErrorCallback(const std::string & error);

  // limits on VESC commands
  struct CommandLimit
  {
    CommandLimit(
      rclcpp::Node * node_ptr,
      const std::string & str,
      const std::experimental::optional<double> & min_lower = std::experimental::optional<double>(),
      const std::experimental::optional<double> & max_upper =
      std::experimental::optional<double>());
    double clip(double value);
    rclcpp::Node * node_ptr;
    rclcpp::Logger logger;
    std::string name;
    std::experimental::optional<double> lower;
    std::experimental::optional<double> upper;
  };

  CommandLimit duty_cycle_limit_;
  CommandLimit current_limit_;
  CommandLimit brake_limit_;
  CommandLimit speed_limit_;
  CommandLimit position_limit_;
  CommandLimit servo_limit_;

  // Known noise floor on this hardware: state.speed (raw ERPM telemetry) sits
  // around 380-400 ERPM even at true rest (empirically observed via live
  // /sensors/core echo -- no net wheel motion, but real current draw and a
  // steady non-zero rpm() reading). Zeroed in vescPacketCallback() below
  // rather than left for every downstream consumer to filter independently --
  // vesc_to_odom_node_backup's own 0.05 m/s deadband is too tight to catch
  // this (~400 erpm / speed_to_erpm_gain is well above it), and filtering
  // once here means any other current or future consumer of /sensors/core
  // gets the same clean signal instead of needing its own copy of this fix.
  double erpm_deadband_;

  // ROS services
  rclcpp::Publisher<VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<VescImuStamped>::SharedPtr imu_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_std_pub_;

  rclcpp::Publisher<Float64>::SharedPtr servo_sensor_pub_;
  rclcpp::SubscriptionBase::SharedPtr duty_cycle_sub_;
  rclcpp::SubscriptionBase::SharedPtr current_sub_;
  rclcpp::SubscriptionBase::SharedPtr brake_sub_;
  rclcpp::SubscriptionBase::SharedPtr speed_sub_;
  rclcpp::SubscriptionBase::SharedPtr position_sub_;
  rclcpp::SubscriptionBase::SharedPtr servo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // driver modes (possible states)
  typedef enum
  {
    MODE_INITIALIZING,
    MODE_OPERATING
  }
  driver_mode_t;

  // other variables
  driver_mode_t driver_mode_;           ///< driver state machine mode (state)
  int fw_version_major_;                ///< firmware major version reported by vesc
  int fw_version_minor_;                ///< firmware minor version reported by vesc
  double gyro_bias_z_;                  ///< static yaw-rate bias (rad/s) subtracted from
                                         ///< gyr_z() before publishing on sensors/imu/raw
  double gyro_scale_x_;                 ///< final gain applied to gyr_x() after bias
                                         ///< subtraction (no bias term exists for x/y
                                         ///< today -- see gyro_bias_z_ above) -- 1.0 =
                                         ///< no-op, placeholder pending same-source
                                         ///< suspicion confirmation for this axis
  double gyro_scale_y_;                 ///< see gyro_scale_x_ -- same, y axis
  double gyro_scale_z_;                 ///< final gain applied to (gyr_z() - gyro_bias_z_)
                                         ///< -- empirically measured live (180deg
                                         ///< reference-rotation test, see that
                                         ///< investigation's own report) to correct a
                                         ///< suspected raw-units mismatch (gyr_z()'s own
                                         ///< "deg/s" comment vs. this field's rad/s
                                         ///< contract) -- 1.0 = no-op if the param is
                                         ///< missing, so existing configs without this
                                         ///< key keep working unchanged

  /** @name Message-level (Level 1) covariance calibration
   *  Measured stationary variance, applied as the diagonal of
   *  angular_velocity_covariance / linear_acceleration_covariance on
   *  sensors/imu/raw. Off-diagonal terms are always 0.0 (a stationary test
   *  doesn't estimate cross-axis correlation). Default 0.0 = uncalibrated.
   *  See f1tenth_diagnostics' sensor_covariance_calibration_node. */
  ///@{
  double gyro_variance_x_, gyro_variance_y_, gyro_variance_z_;
  double accel_variance_x_, accel_variance_y_, accel_variance_z_;
  ///@}

  // ROS callbacks
  void brakeCallback(const Float64::SharedPtr brake);
  void currentCallback(const Float64::SharedPtr current);
  void dutyCycleCallback(const Float64::SharedPtr duty_cycle);
  void positionCallback(const Float64::SharedPtr position);
  void servoCallback(const Float64::SharedPtr servo);
  void speedCallback(const Float64::SharedPtr speed);
  void timerCallback();
};

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_DRIVER_HPP_
