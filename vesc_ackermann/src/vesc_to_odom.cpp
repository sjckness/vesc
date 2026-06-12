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

// ===========================================================================
//  vesc_to_odom_node  --  IMU + VESC odometry fusion (Kalman Filter)
// ===========================================================================
//
//  This node fuses the wheel-odometry produced by the VESC (motor eRPM ->
//  longitudinal speed, commanded steering -> yaw rate via the bicycle model)
//  with the IMU published by the VESC driver on `sensors/imu/raw`
//  (sensor_msgs/Imu) using a small linear Kalman Filter.
//
//      State   x = [ x, y, theta, v, omega ]^T
//      Predict (VESC state callback) : bicycle kinematic model
//      Update  (IMU callback)        : yaw rate (gyro z) and, optionally, yaw
//
// ---------------------------------------------------------------------------
//  # TUNING GUIDE
// ---------------------------------------------------------------------------
//  The filter blends two information sources. How much it trusts each is set
//  by two covariance matrices, both exposed as ROS 2 parameters:
//
//    Q  (process noise, params Q_x, Q_y, Q_theta, Q_v, Q_omega)
//        = how much we DISTRUST the motion (bicycle) model each prediction.
//        * INCREASE Q  -> trust the motion model LESS, let measurements (IMU)
//                          pull the estimate harder / faster.
//        * DECREASE Q  -> trust the motion model MORE, estimate is smoother
//                          but slower to react and more affected by model bias
//                          (e.g. wheel slip, wrong speed_to_erpm_gain).
//
//    R  (measurement noise, params R_omega, R_theta)
//        = how much we DISTRUST the IMU measurement.
//        * INCREASE R  -> trust the IMU LESS (good if the IMU is noisy or its
//                          orientation drifts); estimate leans on the model.
//        * DECREASE R  -> trust the IMU MORE; estimate snaps to the IMU.
//
//  Practical recipe for an F1TENTH car:
//    1. Start with use_imu_orientation=false (VESC IMU yaw can drift). Fuse
//       only the gyro yaw-rate (use_imu_yaw_rate=true).
//    2. Set R_omega to roughly the gyro variance (rad/s)^2. If the VESC IMU
//       filled covariances they are used automatically and R_omega is ignored.
//    3. Set Q_theta / Q_omega so the heading tracks reality without jitter:
//       too-small Q_theta -> sluggish heading that ignores the gyro;
//       too-large  Q_theta -> jittery heading that follows gyro noise.
//    4. Q_x / Q_y mostly affect the reported position covariance; keep small.
//    5. Only enable use_imu_orientation if the IMU yaw is trustworthy
//       (well-calibrated magnetometer / stable fusion on the VESC). Then tune
//       R_theta: small -> heading locks to absolute IMU yaw.
// ---------------------------------------------------------------------------

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std::placeholders::_1;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

// ===========================================================================
//  KalmanFilter5 implementation
// ===========================================================================

KalmanFilter5::KalmanFilter5()
{
  // Initialise covariance to a small diagonal so the filter starts confident
  // about the (zeroed) initial pose but open to correction.
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      P_[i][j] = 0.0;
    }
    P_[i][i] = 0.1;
  }
}

double KalmanFilter5::wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

void KalmanFilter5::predict(
  double v_in, double omega_in, double dt, const std::array<double, N> & q)
{
  // ----- Mean propagation (nonlinear bicycle kinematics) -----------------
  // The VESC supplies the control inputs v_in (from eRPM) and omega_in (from
  // the commanded steering angle). Position/heading are integrated forward;
  // v and omega in the state are set to these control inputs.
  const double theta = x_[THETA];
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);

  x_[X] += v_in * cos_t * dt;
  x_[Y] += v_in * sin_t * dt;
  x_[THETA] = wrapAngle(x_[THETA] + omega_in * dt);
  x_[V] = v_in;
  x_[OMEGA] = omega_in;

  // ----- Covariance propagation:  P = F P F^T + Q ------------------------
  // F is the Jacobian d(f)/d(state) of the propagation above, linearised
  // about the previous state. The v and omega rows are zero because they are
  // overwritten by the (control) inputs each step; their predicted variance
  // therefore comes entirely from Q (how much we distrust the inputs).
  std::array<std::array<double, N>, N> F{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {F[i][j] = 0.0;}
  }
  F[X][X] = 1.0;
  F[X][THETA] = -v_in * sin_t * dt;
  F[Y][Y] = 1.0;
  F[Y][THETA] = v_in * cos_t * dt;
  F[THETA][THETA] = 1.0;
  // F[V][*] and F[OMEGA][*] remain 0 (state v, omega replaced by inputs).

  // tmp = F * P
  std::array<std::array<double, N>, N> tmp{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double s = 0.0;
      for (int k = 0; k < N; ++k) {s += F[i][k] * P_[k][j];}
      tmp[i][j] = s;
    }
  }
  // P = tmp * F^T + Q
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double s = 0.0;
      for (int k = 0; k < N; ++k) {s += tmp[i][k] * F[j][k];}
      P_[i][j] = s;
    }
    P_[i][i] += q[i];
  }
}

void KalmanFilter5::updateScalar(int idx, double z, double r, bool is_angle)
{
  // Scalar Kalman update for a measurement of a single state element.
  // H is the row vector e_idx (a single 1 at position `idx`), so all the
  // matrix products collapse to indexing column `idx` of P.
  //   innovation : y = z - x[idx]
  //   innovation cov : S = P[idx][idx] + r
  //   gain : K[j] = P[j][idx] / S
  //   x += K * y ;  P -= K * (row idx of P)
  double innovation = z - x_[idx];
  if (is_angle) {innovation = wrapAngle(innovation);}

  const double S = P_[idx][idx] + r;
  if (S <= 0.0) {return;}  // guard against degenerate covariance

  std::array<double, N> K{};
  for (int j = 0; j < N; ++j) {
    K[j] = P_[j][idx] / S;
  }

  for (int j = 0; j < N; ++j) {
    x_[j] += K[j] * innovation;
  }
  x_[THETA] = wrapAngle(x_[THETA]);

  // Covariance update: P = (I - K H) P.  H picks row `idx`, so
  // P_new[j][k] = P[j][k] - K[j] * P[idx][k].
  std::array<std::array<double, N>, N> P_new{};
  for (int j = 0; j < N; ++j) {
    for (int k = 0; k < N; ++k) {
      P_new[j][k] = P_[j][k] - K[j] * P_[idx][k];
    }
  }
  P_ = P_new;
}

// ===========================================================================
//  VescToOdom node
// ===========================================================================

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  use_servo_cmd_(true),
  publish_tf_(false),
  kf_initialized_(false)
{
  // ----- existing parameters (unchanged names, for stack compatibility) ---
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);

  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");

  if (use_servo_cmd_) {
    steering_to_servo_gain_ = declare_parameter<double>("steering_angle_to_servo_gain");
    steering_to_servo_offset_ = declare_parameter<double>("steering_angle_to_servo_offset");
    wheelbase_ = declare_parameter<double>("wheelbase");
  }

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // ----- IMU fusion parameters (new) -------------------------------------
  imu_topic_ = declare_parameter<std::string>("imu_topic", "sensors/imu/raw");
  use_imu_yaw_rate_ = declare_parameter<bool>("use_imu_yaw_rate", true);
  use_imu_orientation_ = declare_parameter<bool>("use_imu_orientation", false);

  // Process noise (Q) — see TUNING GUIDE at top of file.
  Q_[KalmanFilter5::X] = declare_parameter<double>("Q_x", 0.01);
  Q_[KalmanFilter5::Y] = declare_parameter<double>("Q_y", 0.01);
  Q_[KalmanFilter5::THETA] = declare_parameter<double>("Q_theta", 0.02);
  Q_[KalmanFilter5::V] = declare_parameter<double>("Q_v", 0.05);
  Q_[KalmanFilter5::OMEGA] = declare_parameter<double>("Q_omega", 0.05);

  // Measurement noise (R) — used when the IMU does not fill its covariances.
  R_omega_ = declare_parameter<double>("R_omega", 0.02);
  R_theta_ = declare_parameter<double>("R_theta", 0.05);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and, optionally, servo command (prediction inputs)
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, _1));
  }

  // subscribe to IMU (measurement update)
  if (use_imu_yaw_rate_ || use_imu_orientation_) {
    imu_sub_ = create_subscription<Imu>(
      imu_topic_, 10, std::bind(&VescToOdom::imuCallback, this, _1));
    RCLCPP_INFO(
      get_logger(), "Fusing IMU from topic '%s' (yaw_rate=%s, orientation=%s)",
      imu_topic_.c_str(),
      use_imu_yaw_rate_ ? "on" : "off",
      use_imu_orientation_ ? "on" : "off");
  } else {
    RCLCPP_INFO(get_logger(), "IMU fusion disabled; running pure bicycle-model odometry.");
  }
}

// --- Prediction step: VESC telemetry drives the bicycle kinematic model ----
void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_) {
    return;
  }

  // convert to engineering units (control inputs for the prediction)
  double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_) {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  // calc elapsed time from message timestamps (never assume a fixed dt)
  double dt = (rclcpp::Time(state->header.stamp) - rclcpp::Time(last_state_->header.stamp)).seconds();
  last_state_ = state;

  // Guard against zero/negative/absurd dt (e.g. first message, clock jumps).
  if (dt <= 0.0 || dt > 1.0) {
    kf_initialized_ = true;
    return;
  }
  kf_initialized_ = true;

  // Kalman prediction with the VESC-derived control inputs.
  kf_.predict(current_speed, current_angular_velocity, dt, Q_);

  publishOdom(state->header.stamp);
}

void VescToOdom::servoCmdCallback(const Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

// --- Update step: IMU corrects yaw rate and (optionally) yaw ---------------
void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  // Wait until the prediction has run at least once so dt/state are sane.
  if (!kf_initialized_) {
    return;
  }

  // Yaw-rate measurement from the gyro (z axis).
  if (use_imu_yaw_rate_) {
    // Use the IMU's own covariance when it is filled, otherwise the param.
    double r = imu->angular_velocity_covariance[8];  // var(omega_z)
    if (r <= 0.0) {r = R_omega_;}
    kf_.updateScalar(KalmanFilter5::OMEGA, imu->angular_velocity.z, r, false);
  }

  // Absolute yaw measurement from the orientation quaternion (optional).
  if (use_imu_orientation_) {
    const auto & q = imu->orientation;
    // Reject an all-zero / unset quaternion.
    const double qnorm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (qnorm > 1e-6) {
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double yaw_meas = std::atan2(siny_cosp, cosy_cosp);

      double r = imu->orientation_covariance[8];  // var(yaw)
      if (r <= 0.0) {r = R_theta_;}
      kf_.updateScalar(KalmanFilter5::THETA, yaw_meas, r, true);
    }
  }

  publishOdom(imu->header.stamp);
}

// --- Publish the fused state as nav_msgs/Odometry (+ optional TF) ----------
void VescToOdom::publishOdom(const rclcpp::Time & stamp)
{
  const double x = kf_.x_[KalmanFilter5::X];
  const double y = kf_.x_[KalmanFilter5::Y];
  const double yaw = kf_.x_[KalmanFilter5::THETA];
  const double v = kf_.x_[KalmanFilter5::V];
  const double omega = kf_.x_[KalmanFilter5::OMEGA];

  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = stamp;
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw / 2.0);
  odom.pose.pose.orientation.w = cos(yaw / 2.0);

  // Pose covariance, filled from the KF covariance P (x, y, yaw blocks).
  // ROS 6x6 row-major order: [x, y, z, roll, pitch, yaw].
  odom.pose.covariance[0] = kf_.P_[KalmanFilter5::X][KalmanFilter5::X];      // x-x
  odom.pose.covariance[1] = kf_.P_[KalmanFilter5::X][KalmanFilter5::Y];      // x-y
  odom.pose.covariance[5] = kf_.P_[KalmanFilter5::X][KalmanFilter5::THETA];  // x-yaw
  odom.pose.covariance[6] = kf_.P_[KalmanFilter5::Y][KalmanFilter5::X];      // y-x
  odom.pose.covariance[7] = kf_.P_[KalmanFilter5::Y][KalmanFilter5::Y];      // y-y
  odom.pose.covariance[11] = kf_.P_[KalmanFilter5::Y][KalmanFilter5::THETA];  // y-yaw
  odom.pose.covariance[30] = kf_.P_[KalmanFilter5::THETA][KalmanFilter5::X];  // yaw-x
  odom.pose.covariance[31] = kf_.P_[KalmanFilter5::THETA][KalmanFilter5::Y];  // yaw-y
  odom.pose.covariance[35] = kf_.P_[KalmanFilter5::THETA][KalmanFilter5::THETA];  // yaw-yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = v;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = omega;

  // Twist covariance from the KF v / omega blocks.
  odom.twist.covariance[0] = kf_.P_[KalmanFilter5::V][KalmanFilter5::V];          // vx-vx
  odom.twist.covariance[35] = kf_.P_[KalmanFilter5::OMEGA][KalmanFilter5::OMEGA];  // wz-wz

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = stamp;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
