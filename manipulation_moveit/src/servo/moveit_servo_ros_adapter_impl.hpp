// Responsibility: Declares the private ROS, TF, cache, and synchronization state of the adapter.

#ifndef SERVO__MOVEIT_SERVO_ROS_ADAPTER_IMPL_HPP_
#define SERVO__MOVEIT_SERVO_ROS_ADAPTER_IMPL_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/msg/servo_status.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"

namespace manipulation::moveit
{

class MoveItServoRosAdapter::Impl final
{
public:
  using Deadline = MoveItServoRosAdapter::Deadline;
  using SteadyClock = std::chrono::steady_clock;

  Impl(rclcpp::Node & node, MoveItServoRosAdapterConfig config);

  [[nodiscard]] bool backend_ready() const noexcept;
  [[nodiscard]] MoveItServoPrepareTargetResult prepare_target(
    const geometry_msgs::msg::PoseStamped & task_goal,
    Deadline deadline);
  [[nodiscard]] MoveItServoRosResult configure_pose_mode(Deadline deadline);
  [[nodiscard]] MoveItServoRosResult set_paused(bool paused, Deadline deadline);
  [[nodiscard]] MoveItServoPublishResult publish_target();
  [[nodiscard]] MoveItServoObservationResult latest_observation(Deadline deadline);

  [[nodiscard]] const MoveItServoRosAdapterConfig & config() const noexcept;

private:
  struct TransformLookupResult
  {
    MoveItServoRosResult status;
    std::optional<geometry_msgs::msg::TransformStamped> transform;
  };

  struct PoseTransformResult
  {
    MoveItServoRosResult status;
    std::optional<geometry_msgs::msg::PoseStamped> pose;
  };

  struct CurrentPoseMeasurement
  {
    builtin_interfaces::msg::Time stamp;
    bool fresh{false};
    bool valid{false};
    bool time_discontinuity{false};
    double position_error_m{0.0};
    double orientation_error_rad{0.0};
  };

  struct JointFeedback
  {
    bool available{false};
    bool valid{false};
    std::uint64_t sequence{0U};
    double max_abs_velocity{0.0};
    builtin_interfaces::msg::Time stamp;
    SteadyClock::time_point received_at{};
  };

  struct StatusFeedback
  {
    bool available{false};
    std::int8_t code{moveit_msgs::msg::ServoStatus::INVALID};
    std::string message;
    SteadyClock::time_point received_at{};
  };

  struct PoseFeedback
  {
    bool available{false};
    bool valid{false};
    bool ros_fresh_at_capture{false};
    std::uint64_t sequence{0U};
    double position_error_m{0.0};
    double orientation_error_rad{0.0};
    builtin_interfaces::msg::Time stamp;
    SteadyClock::time_point received_at{};
  };

  [[nodiscard]] TransformLookupResult lookup_transform_exact(
    const std::string & target_frame,
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    Deadline deadline);
  [[nodiscard]] TransformLookupResult lookup_transform_latest(
    const std::string & target_frame,
    const std::string & source_frame,
    Deadline deadline);
  [[nodiscard]] PoseTransformResult transform_pose_exact(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame,
    Deadline deadline);

  [[nodiscard]] MoveItServoRosResult call_command_type_pose(Deadline deadline);
  [[nodiscard]] MoveItServoRosResult call_pause_service(bool paused, Deadline deadline);

  void observe_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr message);
  void observe_status(moveit_msgs::msg::ServoStatus::ConstSharedPtr message);
  void reset_ros_time_epoch();
  void observe_ros_time_locked(const rclcpp::Time & now);
  void observe_message_stamp_locked(
    const builtin_interfaces::msg::Time & stamp,
    std::optional<std::int64_t> & previous_stamp);
  [[nodiscard]] MoveItServoRosObservation update_and_snapshot_feedback(
    const std::optional<CurrentPoseMeasurement> & current_pose,
    SteadyClock::time_point steady_now);

  [[nodiscard]] static MoveItServoRosResult timeout_result(const std::string & operation);
  [[nodiscard]] static TransformLookupResult transform_failure(
    MoveItServoRosError error, std::string detail);

  rclcpp::Node & node_;
  const MoveItServoRosAdapterConfig config_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr command_type_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

  mutable std::mutex target_mutex_;
  std::optional<MoveItServoPreparedTarget> prepared_target_;

  mutable std::mutex feedback_mutex_;
  JointFeedback joint_feedback_;
  StatusFeedback status_feedback_;
  PoseFeedback pose_feedback_;
  manipulation::servo::ServoBackendState backend_state_{
    manipulation::servo::ServoBackendState::kUnknown};
  std::optional<std::int64_t> last_observed_ros_time_ns_;
  std::optional<std::int64_t> last_joint_stamp_ns_;
  std::optional<std::int64_t> last_tf_stamp_ns_;
  bool ros_time_moved_backwards_{false};

  std::timed_mutex service_mutex_;
};

}  // namespace manipulation::moveit

#endif  // SERVO__MOVEIT_SERVO_ROS_ADAPTER_IMPL_HPP_
