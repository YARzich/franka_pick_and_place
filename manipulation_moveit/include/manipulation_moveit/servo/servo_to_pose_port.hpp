// Responsibility: Defines the narrow MoveIt-facing port consumed by the ServoToPose task.

#ifndef MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_PORT_HPP_
#define MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_PORT_HPP_

#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"

namespace manipulation::moveit
{

/**
 * Replaceable side-effect boundary for one reactive pose-tracking task.
 *
 * Production uses MoveItServoTaskPort. Tests provide an in-memory fake, so the
 * task loop, safe-stop rules, timeout behavior, and pause semantics require no
 * ServoNode, executor, simulation, or hardware.
 */
class ServoToPosePort
{
public:
  using Deadline = std::chrono::steady_clock::time_point;

  virtual ~ServoToPosePort() = default;

  /** Non-blocking, side-effect-free readiness probe for action exposure. */
  [[nodiscard]] virtual bool backend_ready() const noexcept = 0;
  [[nodiscard]] virtual MoveItServoRosResult pause(Deadline deadline) = 0;
  [[nodiscard]] virtual MoveItServoRosResult configure_pose_mode(
    Deadline deadline) = 0;
  [[nodiscard]] virtual MoveItServoPrepareTargetResult prepare_target(
    const geometry_msgs::msg::PoseStamped & target, Deadline deadline) = 0;
  [[nodiscard]] virtual MoveItServoRosResult unpause(Deadline deadline) = 0;
  [[nodiscard]] virtual MoveItServoPublishResult publish_target() = 0;
  [[nodiscard]] virtual MoveItServoObservationResult latest_observation(
    Deadline deadline) = 0;
};

/** Thin forwarding port around the stable MoveIt Servo ROS adapter. */
class MoveItServoTaskPort final : public ServoToPosePort
{
public:
  explicit MoveItServoTaskPort(MoveItServoRosAdapter & adapter) noexcept;

  [[nodiscard]] bool backend_ready() const noexcept override;
  [[nodiscard]] MoveItServoRosResult pause(Deadline deadline) noexcept override;
  [[nodiscard]] MoveItServoRosResult configure_pose_mode(
    Deadline deadline) noexcept override;
  [[nodiscard]] MoveItServoPrepareTargetResult prepare_target(
    const geometry_msgs::msg::PoseStamped & target, Deadline deadline) noexcept override;
  [[nodiscard]] MoveItServoRosResult unpause(Deadline deadline) noexcept override;
  [[nodiscard]] MoveItServoPublishResult publish_target() noexcept override;
  [[nodiscard]] MoveItServoObservationResult latest_observation(
    Deadline deadline) noexcept override;

private:
  MoveItServoRosAdapter & adapter_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_PORT_HPP_
