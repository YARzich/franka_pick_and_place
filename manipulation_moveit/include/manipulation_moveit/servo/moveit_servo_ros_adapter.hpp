// Responsibility: Adapts reactive pose tracking to MoveIt Servo's stable ROS interfaces.

#ifndef MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_HPP_
#define MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_HPP_

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/servo/moveit_servo_ros_adapter_types.hpp"

namespace manipulation::moveit
{

/**
 * ROS-only adapter for MoveIt Servo pose tracking.
 *
 * The adapter deliberately does not own an action server, workflow state
 * machine, resource lease, or robot-specific policy. It borrows a ROS node
 * that must outlive it and exposes bounded primitives used by a higher-level
 * ServoToPose task. The implementation uses only Jazzy message, topic, and
 * service contracts; it does not link the moveit_servo C++ library.
 *
 * A normal session calls pause(), configure_pose_mode(), prepare_target(),
 * unpause(), and then repeatedly publish_target()/latest_observation(). Stop
 * paths stop publishing first and call pause().
 */
class MoveItServoRosAdapter final
{
public:
  using Deadline = std::chrono::steady_clock::time_point;

  MoveItServoRosAdapter(rclcpp::Node & node, MoveItServoRosAdapterConfig config);
  ~MoveItServoRosAdapter();

  MoveItServoRosAdapter(const MoveItServoRosAdapter &) = delete;
  MoveItServoRosAdapter & operator=(const MoveItServoRosAdapter &) = delete;
  MoveItServoRosAdapter(MoveItServoRosAdapter &&) = delete;
  MoveItServoRosAdapter & operator=(MoveItServoRosAdapter &&) = delete;

  /**
   * Return true only after the stable Servo ROS endpoints and fresh robot
   * feedback are present. A previously validated ServoStatus may be stale only
   * while this adapter has positively confirmed the backend pause service.
   *
   * This non-blocking probe lets the action server reject startup-time goals
   * before resource admission. It performs no mode switch, pause, or motion
   * side effect.
   */
  [[nodiscard]] bool backend_ready() const noexcept;

  /**
   * Canonicalize a desired `task_frame` pose into `planning_frame` at the
   * input's exact stamp and derive the corresponding Servo tip target.
   *
   * Any formerly prepared target is cleared before TF work begins, so a failed
   * preparation can never republish an earlier command by accident.
   */
  [[nodiscard]] MoveItServoPrepareTargetResult prepare_target(
    const geometry_msgs::msg::PoseStamped & task_goal,
    Deadline deadline);

  /// Force MoveIt Servo into its POSE command mode through ServoCommandType.
  [[nodiscard]] MoveItServoRosResult configure_pose_mode(Deadline deadline);

  /// Force the backend pause state to false through std_srvs/SetBool.
  [[nodiscard]] MoveItServoRosResult unpause(Deadline deadline);

  /// Publish the prepared tip target with a newly sampled node ROS timestamp.
  [[nodiscard]] MoveItServoPublishResult publish_target();

  /**
   * Read cached status/joints and compute current task-frame pose error.
   *
   * The current planning-to-task TF must be fresh and is acquired within the
   * supplied steady deadline. Position error is Euclidean distance and angular
   * error is the shortest quaternion angle in radians.
   */
  [[nodiscard]] MoveItServoObservationResult latest_observation(Deadline deadline);

  /// Force the backend pause state to true through std_srvs/SetBool.
  [[nodiscard]] MoveItServoRosResult pause(Deadline deadline);

  [[nodiscard]] const MoveItServoRosAdapterConfig & config() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_HPP_
