// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/robot.h>
#include <franka/robot_state.h>

/**
 * RobotPoseController handles Cartesian pose control for a Franka robot.
 * It manages target pose specification, motion generation, and execution.
 */
class RobotPoseController {
 public:
  /**
   * Constructs a RobotPoseController and connects to the robot.
   *
   * @param[in] franka_address IP/hostname of the robot.
   */
  explicit RobotPoseController(const std::string& franka_address);

  /**
   * Destructor closes the robot connection.
   */
  ~RobotPoseController() = default;

  /**
   * Gets the current end effector pose.
   *
   * @return Array of 16 doubles representing the 4x4 transformation matrix.
   */
  std::array<double, 16> getCurrentPose() const;

  /**
   * Gets the current end effector position (x, y, z in meters).
   *
   * @return Array of 3 doubles [x, y, z].
   */
  std::array<double, 3> getCurrentPosition() const;

  /**
   * Gets the current end effector orientation as quaternion.
   *
   * @return Array of 4 doubles [x, y, z, w].
   */
  std::array<double, 4> getCurrentOrientation() const;

  /**
   * Sets the target pose using position and orientation.
   *
   * @param[in] position Target position [x, y, z] in meters.
   * @param[in] orientation Target orientation as quaternion [x, y, z, w].
   */
  void setTargetPose(const std::array<double, 3>& position,
                     const std::array<double, 4>& orientation);

  /**
   * Sets the Cartesian velocity for motion.
   *
   * @param[in] velocity_mm_s Desired velocity in mm/s.
   */
  void setVelocity(double velocity_mm_s);

  /**
   * Gets the calculated movement duration in seconds.
   *
   * @return Movement duration based on distance and velocity.
   */
  double getMovementDuration() const { return movement_duration_; }

  /**
   * Gets the distance to target in millimeters.
   *
   * @return Distance in mm.
   */
  double getDistanceToTarget() const { return distance_to_target_; }

  /**
   * Executes the motion to the target pose.
   *
   * @throw franka::Exception if motion control fails.
   */
  void executeMotion();

  /**
   * Gets whether the motion completed successfully.
   *
   * @return True if motion finished, false otherwise.
   */
  bool isMotionFinished() const { return motion_finished_; }

 private:
  std::unique_ptr<franka::Robot> robot_;
  std::array<double, 16> target_pose_;
  std::array<double, 3> target_position_;
  std::array<double, 4> target_orientation_;
  double velocity_mm_s_;
  double movement_duration_;
  double distance_to_target_;
  bool motion_finished_;

  /**
   * Converts quaternion to transformation matrix.
   *
   * @param[in] position Position vector [x, y, z].
   * @param[in] orientation Quaternion [x, y, z, w].
   * @return 4x4 transformation matrix as array of 16 doubles.
   */
  static std::array<double, 16> quaternionToTransformationMatrix(
      const std::array<double, 3>& position,
      const std::array<double, 4>& orientation);

  /**
   * Calculates movement duration based on distance and velocity.
   */
  void calculateMovementDuration();

  /**
   * Sets default robot impedance and behavior.
   */
  void setDefaultBehavior();
};
