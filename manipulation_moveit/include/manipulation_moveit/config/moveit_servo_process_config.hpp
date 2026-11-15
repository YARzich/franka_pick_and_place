// Responsibility: Defines the fully validated ServoToPose process configuration.

#ifndef MANIPULATION_MOVEIT__CONFIG__MOVEIT_SERVO_PROCESS_CONFIG_HPP_
#define MANIPULATION_MOVEIT__CONFIG__MOVEIT_SERVO_PROCESS_CONFIG_HPP_

#include <string>

#include "manipulation/skill/preset_identity.hpp"
#include "manipulation_moveit/servo/moveit_servo_ros_adapter_types.hpp"
#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation::moveit
{

/** One immutable ServoToPose policy and its concrete ROS/robot bindings. */
struct LoadedMoveItServoProcessConfig
{
  manipulation::skill::PresetIdentity preset_identity;
  ServoToPoseActionServerConfig action_server;
  MoveItServoRosAdapterConfig adapter;
  std::string arm_resource_id;
};

/** Declare and validate the complete required ServoToPose process snapshot. */
[[nodiscard]] LoadedMoveItServoProcessConfig
load_moveit_servo_process_config(rclcpp::Node & node);

/** Fingerprint robot binding and reactive motion policy, excluding ROS deployment names. */
[[nodiscard]] manipulation::skill::PresetIdentity
make_moveit_servo_preset_identity(
  const LoadedMoveItServoProcessConfig & config);

/** Fingerprint the Servo action facade and all concrete ROS adapter wiring. */
[[nodiscard]] manipulation::skill::PresetIdentity
make_moveit_servo_deployment_identity(
  const LoadedMoveItServoProcessConfig & config);

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__CONFIG__MOVEIT_SERVO_PROCESS_CONFIG_HPP_
