// Responsibility: Defines the validated configuration and typed observations of the Servo ROS API.

#ifndef MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_TYPES_HPP_
#define MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_TYPES_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "manipulation/servo/servo_to_pose_types.hpp"

namespace manipulation::moveit
{

/** ROS graph endpoints exposed by MoveIt Servo's stable Jazzy interface. */
struct MoveItServoRosEndpoints
{
  std::string pose_command_topic{"/servo_node/pose_target_cmds"};
  std::string command_type_service{"/servo_node/switch_command_type"};
  std::string pause_service{"/servo_node/pause_servo"};
  std::string status_topic{"/servo_node/status"};
  std::string joint_state_topic{"/joint_states"};
};

/**
 * Immutable construction policy for one MoveIt Servo ROS adapter.
 *
 * `task_frame` is the controlled application frame whose target the caller
 * supplies. `servo_tip_frame` is the link expected by MoveIt Servo. Their TF
 * offset is evaluated at the target's exact stamp instead of being copied into
 * robot-specific code. Freshness durations use a steady clock so a paused or
 * reset simulation clock cannot extend a wait.
 */
struct MoveItServoRosAdapterConfig
{
  std::string planning_frame;
  std::string task_frame;
  std::string servo_tip_frame;
  std::vector<std::string> joint_names;
  MoveItServoRosEndpoints endpoints;
  std::chrono::steady_clock::duration poll_period{std::chrono::milliseconds{5}};
  std::chrono::steady_clock::duration current_tf_max_age{std::chrono::milliseconds{250}};
  std::chrono::steady_clock::duration joint_state_max_age{std::chrono::milliseconds{250}};
  std::chrono::steady_clock::duration status_max_age{std::chrono::milliseconds{500}};
};

/// Return no value only when every frame, joint, endpoint, and duration is usable.
[[nodiscard]] std::optional<std::string> validate_moveit_servo_ros_adapter_config(
  const MoveItServoRosAdapterConfig & config);

/// Failure categories of bounded ROS and TF adapter operations.
enum class MoveItServoRosError : std::uint8_t
{
  kNone,
  kUnavailable,
  kTimeout,
  kInvalidData,
  kRejected,
  kTimeDiscontinuity,
  kInternalError,
};

/** Typed status for one synchronous adapter operation. */
class MoveItServoRosResult final
{
public:
  MoveItServoRosResult() = default;

  [[nodiscard]] static MoveItServoRosResult success(std::string detail = {});
  [[nodiscard]] static MoveItServoRosResult failure(
    MoveItServoRosError error, std::string detail);

  [[nodiscard]] bool succeeded() const noexcept;
  [[nodiscard]] MoveItServoRosError error() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  MoveItServoRosResult(MoveItServoRosError error, std::string detail);

  MoveItServoRosError error_{MoveItServoRosError::kInternalError};
  std::string detail_{"Servo ROS result was not initialized"};
};

/** Exact-stamp task target plus the corresponding MoveIt Servo tip target. */
struct MoveItServoPreparedTarget
{
  geometry_msgs::msg::PoseStamped task_goal;
  geometry_msgs::msg::PoseStamped servo_tip_goal;
};

struct MoveItServoPrepareTargetResult
{
  MoveItServoRosResult status;
  std::optional<MoveItServoPreparedTarget> target;
};

struct MoveItServoPublishResult
{
  MoveItServoRosResult status;
  std::optional<geometry_msgs::msg::PoseStamped> published_target;
};

/** Cached raw MoveIt Servo status with local steady-clock freshness. */
struct MoveItServoStatusSnapshot
{
  bool available{false};
  bool fresh{false};
  bool valid{false};
  std::int8_t code{-1};
  std::string message;
};

/**
 * One adapter observation ready for the backend-neutral ServoToPose machine.
 *
 * Pose and velocity sequences advance only for distinct stamped samples. A
 * stale callback read therefore cannot satisfy a multi-sample completion or
 * stopped-robot policy by being polled repeatedly. Joint feedback must be
 * fresh both by local receive time and by its ROS header stamp.
 */
struct MoveItServoRosObservation
{
  manipulation::servo::ServoBackendState backend_state{
    manipulation::servo::ServoBackendState::kUnknown};
  manipulation::servo::ServoSafetyStatus safety_status{
    manipulation::servo::ServoSafetyStatus::kStopRequired};
  manipulation::servo::ServoPoseSample pose_sample;
  manipulation::servo::ServoVelocitySample velocity_sample;
  MoveItServoStatusSnapshot status;
  bool ros_time_moved_backwards{false};
};

struct MoveItServoObservationResult
{
  MoveItServoRosResult status;
  MoveItServoRosObservation observation;
};

/** Fail-closed mapping from the stable moveit_msgs/ServoStatus code contract. */
[[nodiscard]] manipulation::servo::ServoSafetyStatus classify_moveit_servo_status(
  std::int8_t code) noexcept;

/** Return true only for status codes defined by moveit_msgs/msg/ServoStatus. */
[[nodiscard]] bool is_known_moveit_servo_status(std::int8_t code) noexcept;

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__SERVO__MOVEIT_SERVO_ROS_ADAPTER_TYPES_HPP_
