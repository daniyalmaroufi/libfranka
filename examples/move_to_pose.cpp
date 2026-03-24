// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"

/**
 * @example move_to_pose.cpp
 * An example showing how to move the robot to a user-specified Cartesian pose.
 * Accepts position (x, y, z) and orientation (x, y, z, w as quaternion) from the user.
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 */

// Convert quaternion (x, y, z, w) to 4x4 rotation matrix
std::array<double, 16> quaternionToTransformationMatrix(
    double pos_x, double pos_y, double pos_z,
    double quat_x, double quat_y, double quat_z, double quat_w) {
  // Create Eigen quaternion (note: Eigen uses w, x, y, z order)
  Eigen::Quaterniond q(quat_w, quat_x, quat_y, quat_z);
  
  // Convert to rotation matrix
  Eigen::Matrix3d rotation = q.toRotationMatrix();
  
  // Create 4x4 transformation matrix (column-major order for libfranka)
  std::array<double, 16> T_EE = {};
  
  // Fill rotation part (column-major)
  T_EE[0] = rotation(0, 0);
  T_EE[1] = rotation(1, 0);
  T_EE[2] = rotation(2, 0);
  
  T_EE[4] = rotation(0, 1);
  T_EE[5] = rotation(1, 1);
  T_EE[6] = rotation(2, 1);
  
  T_EE[8] = rotation(0, 2);
  T_EE[9] = rotation(1, 2);
  T_EE[10] = rotation(2, 2);
  
  // Fill position part
  T_EE[12] = pos_x;
  T_EE[13] = pos_y;
  T_EE[14] = pos_z;
  
  // Fill bottom row
  T_EE[15] = 1.0;
  
  return T_EE;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    
    // Try to set default behavior, but continue if it fails (e.g., in Reflex mode)
    try {
      setDefaultBehavior(robot);
    } catch (const franka::Exception& e) {
      std::cout << "Warning: Could not set default behavior: " << e.what() << std::endl;
      std::cout << "Continuing anyway, but robot may not be fully configured." << std::endl;
      std::cout << "If in Reflex mode, exit it on the robot before proceeding." << std::endl << std::endl;
    }

    // Get and display current robot pose
    franka::RobotState initial_state = robot.readOnce();
    std::cout << "Current end effector pose:" << std::endl;
    std::cout << "  Position: [" << initial_state.O_T_EE_c[12] << ", " 
              << initial_state.O_T_EE_c[13] << ", " << initial_state.O_T_EE_c[14] << "]" << std::endl;
    
    // Convert rotation matrix to quaternion for display
    Eigen::Matrix3d current_rotation = Eigen::Map<const Eigen::Matrix4d>(initial_state.O_T_EE_c.data()).block<3, 3>(0, 0);
    Eigen::Quaterniond current_quat(current_rotation);
    std::cout << "  Quaternion: [" << current_quat.x() << ", " << current_quat.y() << ", " 
              << current_quat.z() << ", " << current_quat.w() << "]" << std::endl << std::endl;

    // Get position from user
    double pos_x, pos_y, pos_z;
    std::cout << "Enter end effector position:" << std::endl;
    std::cout << "  X (meters): ";
    if (!(std::cin >> pos_x)) {
      std::cerr << "Error: Invalid input for X position" << std::endl;
      return -1;
    }
    std::cout << "  Y (meters): ";
    if (!(std::cin >> pos_y)) {
      std::cerr << "Error: Invalid input for Y position" << std::endl;
      return -1;
    }
    std::cout << "  Z (meters): ";
    if (!(std::cin >> pos_z)) {
      std::cerr << "Error: Invalid input for Z position" << std::endl;
      return -1;
    }

    // Get orientation from user or use current orientation
    double quat_x, quat_y, quat_z, quat_w;
    std::cout << "\nUse current orientation? (y/n, default: y): ";
    std::string use_current;
    std::cin.ignore();
    std::getline(std::cin, use_current);
    
    if (use_current.empty() || use_current[0] == 'y' || use_current[0] == 'Y') {
      // Use current orientation
      quat_x = current_quat.x();
      quat_y = current_quat.y();
      quat_z = current_quat.z();
      quat_w = current_quat.w();
      std::cout << "Using current orientation: [" << quat_x << ", " << quat_y << ", " 
                << quat_z << ", " << quat_w << "]" << std::endl;
    } else {
      // Enter new orientation
      std::cout << "\nEnter end effector orientation (as quaternion):" << std::endl;
      std::cout << "  X: ";
      if (!(std::cin >> quat_x)) {
        std::cerr << "Error: Invalid input for quaternion X" << std::endl;
        return -1;
      }
      std::cout << "  Y: ";
      if (!(std::cin >> quat_y)) {
        std::cerr << "Error: Invalid input for quaternion Y" << std::endl;
        return -1;
      }
      std::cout << "  Z: ";
      if (!(std::cin >> quat_z)) {
        std::cerr << "Error: Invalid input for quaternion Z" << std::endl;
        return -1;
      }
      std::cout << "  W: ";
      if (!(std::cin >> quat_w)) {
        std::cerr << "Error: Invalid input for quaternion W" << std::endl;
        return -1;
      }
    }

    // Normalize quaternion
    double norm = std::sqrt(quat_x*quat_x + quat_y*quat_y + quat_z*quat_z + quat_w*quat_w);
    if (norm < 1e-6) {
      std::cerr << "Error: Invalid quaternion (magnitude too small)" << std::endl;
      return -1;
    }
    quat_x /= norm;
    quat_y /= norm;
    quat_z /= norm;
    quat_w /= norm;

    std::cout << "\nTarget Pose:" << std::endl;
    std::cout << "  Position: [" << pos_x << ", " << pos_y << ", " << pos_z << "]" << std::endl;
    std::cout << "  Quaternion: [" << quat_x << ", " << quat_y << ", " << quat_z << ", " << quat_w << "]" << std::endl;

    // Calculate distance to target
    double dx = pos_x - initial_state.O_T_EE_c[12];
    double dy = pos_y - initial_state.O_T_EE_c[13];
    double dz = pos_z - initial_state.O_T_EE_c[14];
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    std::cout << "Distance to target: " << distance * 1000.0 << " mm" << std::endl;

    // Get Cartesian velocity from user (in mm/s)
    double velocity_mm_s = 50.0;  // default 50 mm/s
    std::cout << "\nEnter desired Cartesian velocity in mm/s (default 50.0): ";
    std::string velocity_input;
    std::getline(std::cin, velocity_input);
    if (!velocity_input.empty()) {
      try {
        velocity_mm_s = std::stod(velocity_input);
        if (velocity_mm_s <= 0) {
          std::cerr << "Invalid velocity, using default 50.0 mm/s" << std::endl;
          velocity_mm_s = 50.0;
        }
      } catch (const std::exception& e) {
        std::cerr << "Invalid input, using default 50.0 mm/s" << std::endl;
        velocity_mm_s = 50.0;
      }
    }
    std::cout << "Cartesian velocity: " << velocity_mm_s << " mm/s" << std::endl;

    // Calculate movement duration based on distance and velocity
    double velocity_m_s = velocity_mm_s / 1000.0;  // convert to m/s
    double calculated_duration = (distance > 1e-6) ? (distance / velocity_m_s) : 5.0;
    // Add significant safety buffer (1.5x multiplier) to ensure smooth acceleration/deceleration
    // The trapezoidal profile needs extra time to avoid velocity discontinuities
    double movement_duration = calculated_duration * 1.5;
    movement_duration = std::max(movement_duration, 8.0);  // Ensure at least 8 seconds
    std::cout << "Calculated movement duration: " << movement_duration << " seconds" << std::endl;

    // Set lower impedance for smoother motion
    try {
      // Set Cartesian impedance (stiffness): lower values for smoother, less jerky motion
      robot.setCartesianImpedance({500.0, 500.0, 500.0, 50.0, 50.0, 50.0});
      
      // Set joint impedance (stiffness): lower values for smoother motion
      robot.setJointImpedance({1500, 1500, 1500, 1250, 1250, 1000, 1000});
    } catch (const franka::Exception& e) {
      std::cout << "Warning: Could not set impedance: " << e.what() << std::endl;
    }

    // Create target transformation matrix
    std::array<double, 16> target_pose = quaternionToTransformationMatrix(
        pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w);

    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Also ensure the robot is activated (power button on teach pendant)." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::cin.ignore();

    // Move to target pose
    double time = 0.0;
    bool motion_finished = false;
    double target_reached_time = -1.0;  // Time when target was first reached
    constexpr double settling_time = 0.25;  // 0.25 second settling time after reaching target
    
    try {
      robot.control([&time, &target_pose, movement_duration, &motion_finished, &target_reached_time](const franka::RobotState& robot_state,
                                          franka::Duration period) -> franka::CartesianPose {
        time += period.toSec();
        
        // Trapezoidal velocity profile for smoother motion
        double alpha;
        double accel_time = movement_duration * 0.2;  // 20% acceleration, 60% constant velocity, 20% deceleration
        double decel_start = movement_duration * 0.8;
        
        if (time <= accel_time) {
          // Acceleration phase: quadratic ramp
          alpha = 0.5 * (time / accel_time) * (time / accel_time);
        } else if (time <= decel_start) {
          // Constant velocity phase
          alpha = 0.5 + (time - accel_time) / movement_duration * 0.4;
        } else {
          // Deceleration phase: quadratic ramp
          double t_decel = (time - decel_start) / accel_time;
          alpha = 0.9 + 0.1 * (1.0 - (1.0 - t_decel) * (1.0 - t_decel));
        }
        
        alpha = std::min(alpha, 1.0);

        std::array<double, 16> intermediate_pose;
        
        // Interpolate position
        for (size_t i = 0; i < 3; i++) {
          intermediate_pose[12 + i] = robot_state.O_T_EE_c[12 + i] * (1 - alpha) + 
                                      target_pose[12 + i] * alpha;
        }
        
        // Interpolate rotation using SLERP (spherical linear interpolation)
        Eigen::Quaterniond current_q(
          Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE_c.data()).block<3, 3>(0, 0)
        );
        
        Eigen::Quaterniond target_q(
          Eigen::Map<const Eigen::Matrix4d>(target_pose.data()).block<3, 3>(0, 0)
        );
        
        Eigen::Quaterniond interpolated_q = current_q.slerp(alpha, target_q);
        Eigen::Matrix3d interpolated_rotation = interpolated_q.toRotationMatrix();
        
        // Fill rotation part
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

        // Check if robot has reached the target pose
        double pos_error = 0.0;
        for (size_t i = 0; i < 3; i++) {
          double diff = robot_state.O_T_EE_c[12 + i] - target_pose[12 + i];
          pos_error += diff * diff;
        }
        pos_error = std::sqrt(pos_error);
        
        // Get rotation error
        Eigen::Quaterniond actual_q(
          Eigen::Map<const Eigen::Matrix4d>(robot_state.O_T_EE_c.data()).block<3, 3>(0, 0)
        );
        Eigen::Quaterniond target_q_final(
          Eigen::Map<const Eigen::Matrix4d>(target_pose.data()).block<3, 3>(0, 0)
        );
        double rot_error = 1.0 - std::abs(actual_q.dot(target_q_final));
        
        // Record when target is first reached (20mm position tolerance, past acceleration phase)
        if (target_reached_time < 0.0 && pos_error < 0.020 && rot_error < 0.05 && time > accel_time) {
          target_reached_time = time;
        }

        // Exit with settling time after reaching target, or at full duration
        if (target_reached_time >= 0.0 && (time >= target_reached_time + settling_time)) {
          std::cout << std::endl << "Target pose reached and settled" << std::endl;
          motion_finished = true;
          return franka::MotionFinished(target_pose);
        }
        
        if (time >= movement_duration) {
          std::cout << std::endl << "Finished moving to target pose" << std::endl;
          motion_finished = true;
          return franka::MotionFinished(target_pose);
        }
        return intermediate_pose;
      });
      
      if (motion_finished) {
        std::cout << "Motion completed successfully" << std::endl;
      }
    } catch (const franka::Exception& e) {
      std::cerr << "Motion control error: " << e.what() << std::endl;
      std::cerr << "\nTroubleshooting:" << std::endl;
      std::cerr << "  - Try increasing the movement duration (slower motion)" << std::endl;
      std::cerr << "  - Check if the target pose is reachable by the robot" << std::endl;
      std::cerr << "  - Ensure the robot is not near joint limits" << std::endl;
      std::cerr << "  - The robot may have encountered a collision or safety issue" << std::endl;
      return -1;
    }

  } catch (const franka::Exception& e) {
    std::string error_msg = e.what();
    std::cerr << "Franka exception: " << error_msg << std::endl;
    
    // Provide specific guidance based on error type
    if (error_msg.find("User stopped") != std::string::npos) {
      std::cerr << "\nRobot is in 'User stopped' mode. Please:" << std::endl;
      std::cerr << "  1. Check the teach pendant for any active stop buttons" << std::endl;
      std::cerr << "  2. Press the power button on the teach pendant to activate the robot" << std::endl;
      std::cerr << "  3. Ensure no emergency stop is active" << std::endl;
    } else if (error_msg.find("Reflex") != std::string::npos) {
      std::cerr << "\nRobot is in Reflex mode. Please:" << std::endl;
      std::cerr << "  1. Exit Reflex mode on the robot's teach pendant or Desk" << std::endl;
      std::cerr << "  2. Manually move the end effector away from obstacles" << std::endl;
    } else {
      std::cerr << "\nPlease check:" << std::endl;
      std::cerr << "  - Robot is powered on and activated" << std::endl;
      std::cerr << "  - Robot is not in Reflex or User stopped mode" << std::endl;
      std::cerr << "  - Network connection is stable" << std::endl;
      std::cerr << "  - Hostname/IP is correct" << std::endl;
      std::cerr << "  - Target pose is reachable by the robot" << std::endl;
    }
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Standard exception: " << e.what() << std::endl;
    return -1;
  } catch (...) {
    std::cerr << "Unknown error occurred" << std::endl;
    return -1;
  }

  return 0;
}
