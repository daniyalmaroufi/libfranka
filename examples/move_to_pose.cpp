// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <array>
#include <string>

#include "robot_pose_controller.h"

/**
 * @example move_to_pose.cpp
 * An example showing how to move the robot to a user-specified Cartesian pose.
 * Accepts position (x, y, z) and orientation (x, y, z, w as quaternion) from the user.
 *
 * @warning Before executing this example, make sure there is enough space around the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // Create robot controller
    RobotPoseController controller(argv[1]);

    // Get and display current robot pose
    auto current_pos = controller.getCurrentPosition();
    auto current_orient = controller.getCurrentOrientation();

    std::cout << "Current end effector pose:" << std::endl;
    std::cout << "  Position: [" << current_pos[0] << ", " << current_pos[1] << ", " << current_pos[2]
              << "]" << std::endl;
    std::cout << "  Quaternion: [" << current_orient[0] << ", " << current_orient[1] << ", "
              << current_orient[2] << ", " << current_orient[3] << "]" << std::endl
              << std::endl;

    // Get target position from user
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

    // Get orientation from user or use current
    double quat_x, quat_y, quat_z, quat_w;
    std::cout << "\nUse current orientation? (y/n, default: y): ";
    std::string use_current;
    std::cin.ignore();
    std::getline(std::cin, use_current);

    if (use_current.empty() || use_current[0] == 'y' || use_current[0] == 'Y') {
      quat_x = current_orient[0];
      quat_y = current_orient[1];
      quat_z = current_orient[2];
      quat_w = current_orient[3];
      std::cout << "Using current orientation: [" << quat_x << ", " << quat_y << ", " << quat_z
                << ", " << quat_w << "]" << std::endl;
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
    double norm = std::sqrt(quat_x * quat_x + quat_y * quat_y + quat_z * quat_z + quat_w * quat_w);
    if (norm < 1e-6) {
      std::cerr << "Error: Invalid quaternion (magnitude too small)" << std::endl;
      return -1;
    }
    quat_x /= norm;
    quat_y /= norm;
    quat_z /= norm;
    quat_w /= norm;

    // Display target pose
    std::cout << "\nTarget Pose:" << std::endl;
    std::cout << "  Position: [" << pos_x << ", " << pos_y << ", " << pos_z << "]" << std::endl;
    std::cout << "  Quaternion: [" << quat_x << ", " << quat_y << ", " << quat_z << ", " << quat_w
              << "]" << std::endl;

    // Set target pose
    std::array<double, 3> target_position = {pos_x, pos_y, pos_z};
    std::array<double, 4> target_orientation = {quat_x, quat_y, quat_z, quat_w};
    controller.setTargetPose(target_position, target_orientation);

    std::cout << "Distance to target: " << controller.getDistanceToTarget() << " mm" << std::endl;

    // Get Cartesian velocity from user
    double velocity_mm_s = 50.0;
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

    controller.setVelocity(velocity_mm_s);
    std::cout << "Cartesian velocity: " << velocity_mm_s << " mm/s" << std::endl;
    std::cout << "Calculated movement duration: " << controller.getMovementDuration() << " seconds"
              << std::endl;

    // Confirm before moving
    std::cout << "\nWARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Also ensure the robot is activated (power button on teach pendant)." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::cin.ignore();

    // Execute motion
    controller.executeMotion();

    if (controller.isMotionFinished()) {
      std::cout << "Motion completed successfully" << std::endl;
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
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  } catch (...) {
    std::cerr << "Unknown error occurred" << std::endl;
    return -1;
  }

  return 0;
}
