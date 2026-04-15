// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "robot_pose_controller.h"
#include <cmath>
#include <iostream>
#include "examples_common.h"

RobotPoseController::RobotPoseController(const std::string& franka_address)
    : velocity_mm_s_(50.0), movement_duration_(0.0), distance_to_target_(0.0), motion_finished_(false) {
  robot_ = std::make_unique<franka::Robot>(franka_address);
  setDefaultBehavior();
}

std::array<double, 16> RobotPoseController::getCurrentPose() const {
  franka::RobotState state = robot_->readOnce();
  return state.O_T_EE_c;
}

std::array<double, 3> RobotPoseController::getCurrentPosition() const {
  franka::RobotState state = robot_->readOnce();
  return {state.O_T_EE_c[12], state.O_T_EE_c[13], state.O_T_EE_c[14]};
}

std::array<double, 4> RobotPoseController::getCurrentOrientation() const {
  franka::RobotState state = robot_->readOnce();
  Eigen::Matrix3d rotation = Eigen::Map<const Eigen::Matrix4d>(state.O_T_EE_c.data()).block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rotation);
  return {quat.x(), quat.y(), quat.z(), quat.w()};
}

void RobotPoseController::setTargetPose(const std::array<double, 3>& position,
                                         const std::array<double, 4>& orientation) {
  target_position_ = position;
  target_orientation_ = orientation;
  target_pose_ = quaternionToTransformationMatrix(position, orientation);

  // Calculate distance to target
  franka::RobotState state = robot_->readOnce();
  double dx = position[0] - state.O_T_EE_c[12];
  double dy = position[1] - state.O_T_EE_c[13];
  double dz = position[2] - state.O_T_EE_c[14];
  distance_to_target_ = std::sqrt(dx * dx + dy * dy + dz * dz);

  calculateMovementDuration();
}

void RobotPoseController::setVelocity(double velocity_mm_s) {
  if (velocity_mm_s <= 0) {
    throw std::invalid_argument("Velocity must be positive");
  }
  velocity_mm_s_ = velocity_mm_s;
  calculateMovementDuration();
}

void RobotPoseController::executeMotion() {
  motion_finished_ = false;
  double time = 0.0;
  double target_reached_time = -1.0;
  constexpr double settling_time = 0.25;

  robot_->control([this, &time, &target_reached_time](const franka::RobotState& robot_state,
                                                       franka::Duration period) -> franka::CartesianPose {
    time += period.toSec();

    // Trapezoidal velocity profile
    double alpha;
    double accel_time = movement_duration_ * 0.2;
    double decel_start = movement_duration_ * 0.8;

    if (time <= accel_time) {
      alpha = 0.5 * (time / accel_time) * (time / accel_time);
    } else if (time <= decel_start) {
      alpha = 0.5 + (time - accel_time) / movement_duration_ * 0.4;
    } else {
      double t_decel = (time - decel_start) / accel_time;
      alpha = 0.9 + 0.1 * (1.0 - (1.0 - t_decel) * (1.0 - t_decel));
    }

    alpha = std::min(alpha, 1.0);

    std::array<double, 16> intermediate_pose;

    // Interpolate position
    for (size_t i = 0; i < 3; i++) {
      intermediate_pose[12 + i] = robot_state.O_T_EE_c[12 + i] * (1 - alpha) + target_pose_[12 + i] * alpha;
    }

    // Interpolate rotation using SLERP
    Eigen::Quaterniond current_q(
        Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE_c.data()).block<3, 3>(0, 0));

    Eigen::Quaterniond target_q(
        Eigen::Map<const Eigen::Matrix4d>(target_pose_.data()).block<3, 3>(0, 0));

    Eigen::Quaterniond interpolated_q = current_q.slerp(alpha, target_q);
    Eigen::Matrix3d interpolated_rotation = interpolated_q.toRotationMatrix();

    intermediate_pose[0] = interpolated_rotation(0, 0);
    intermediate_pose[1] = interpolated_rotation(1, 0);
    intermediate_pose[2] = interpolated_rotation(2, 0);

    intermediate_pose[4] = interpolated_rotation(0, 1);
    intermediate_pose[5] = interpolated_rotation(1, 1);
    intermediate_pose[6] = interpolated_rotation(2, 1);

    intermediate_pose[8] = interpolated_rotation(0, 2);
    intermediate_pose[9] = interpolated_rotation(1, 2);
    intermediate_pose[10] = interpolated_rotation(2, 2);

    intermediate_pose[3] = 0.0;
    intermediate_pose[7] = 0.0;
    intermediate_pose[11] = 0.0;
    intermediate_pose[15] = 1.0;

    // Check if target is reached
    double pos_error = 0.0;
    for (size_t i = 0; i < 3; i++) {
      double diff = robot_state.O_T_EE_c[12 + i] - target_pose_[12 + i];
      pos_error += diff * diff;
    }
    pos_error = std::sqrt(pos_error);

    Eigen::Quaterniond actual_q(
        Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE_c.data()).block<3, 3>(0, 0));
    Eigen::Quaterniond target_q_final(
        Eigen::Map<const Eigen::Matrix4d>(target_pose_.data()).block<3, 3>(0, 0));
    double rot_error = 1.0 - std::abs(actual_q.dot(target_q_final));

    if (target_reached_time < 0.0 && pos_error < 0.020 && rot_error < 0.05 && time > accel_time) {
      target_reached_time = time;
    }

    if (target_reached_time >= 0.0 && (time >= target_reached_time + settling_time)) {
      motion_finished_ = true;
      return franka::MotionFinished(target_pose_);
    }

    if (time >= movement_duration_) {
      motion_finished_ = true;
      return franka::MotionFinished(target_pose_);
    }

    return intermediate_pose;
  });
}

std::array<double, 16> RobotPoseController::quaternionToTransformationMatrix(
    const std::array<double, 3>& position, const std::array<double, 4>& orientation) {
  Eigen::Quaterniond q(orientation[3], orientation[0], orientation[1], orientation[2]);
  Eigen::Matrix3d rotation = q.toRotationMatrix();

  std::array<double, 16> T_EE = {};

  T_EE[0] = rotation(0, 0);
  T_EE[1] = rotation(1, 0);
  T_EE[2] = rotation(2, 0);

  T_EE[4] = rotation(0, 1);
  T_EE[5] = rotation(1, 1);
  T_EE[6] = rotation(2, 1);

  T_EE[8] = rotation(0, 2);
  T_EE[9] = rotation(1, 2);
  T_EE[10] = rotation(2, 2);

  T_EE[12] = position[0];
  T_EE[13] = position[1];
  T_EE[14] = position[2];

  T_EE[15] = 1.0;

  return T_EE;
}

void RobotPoseController::calculateMovementDuration() {
  double velocity_m_s = velocity_mm_s_ / 1000.0;
  double calculated_duration = (distance_to_target_ > 1e-6) ? (distance_to_target_ / velocity_m_s) : 5.0;

  movement_duration_ = calculated_duration * 1.5;
  movement_duration_ = std::max(movement_duration_, 8.0);
}

void RobotPoseController::setDefaultBehavior() {
  try {
    ::setDefaultBehavior(*robot_);
  } catch (const franka::Exception& e) {
    std::cout << "Warning: Could not set default behavior: " << e.what() << std::endl;
  }

  try {
    robot_->setCartesianImpedance({500.0, 500.0, 500.0, 50.0, 50.0, 50.0});
    robot_->setJointImpedance({1500, 1500, 1500, 1250, 1250, 1000, 1000});
  } catch (const franka::Exception& e) {
    std::cout << "Warning: Could not set impedance: " << e.what() << std::endl;
  }
}
