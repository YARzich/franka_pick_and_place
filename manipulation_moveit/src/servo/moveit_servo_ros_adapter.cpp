// Responsibility: Constructs the Servo ROS wiring and exposes the small synchronous adapter API.

#include "manipulation_moveit/servo/moveit_servo_ros_adapter.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "tf2_ros/qos.hpp"

#include "manipulation/validation/rigid_pose.hpp"
#include "moveit_servo_ros_adapter_impl.hpp"

namespace manipulation::moveit
{

MoveItServoRosAdapter::Impl::Impl(
  rclcpp::Node & node, MoveItServoRosAdapterConfig config)
: node_(node),
  config_(std::move(config)),
  callback_group_(node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  tf_buffer_(node_.get_clock())
{
  if (const auto error = validate_moveit_servo_ros_adapter_config(config_)) {
    throw std::invalid_argument("invalid MoveIt Servo ROS adapter config: " + *error);
  }

  rclcpp::SubscriptionOptions dynamic_options;
  dynamic_options.callback_group = callback_group_;
  rclcpp::SubscriptionOptions static_options;
  static_options.callback_group = callback_group_;
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    tf_buffer_,
    node_.get_node_base_interface(),
    node_.get_node_logging_interface(),
    node_.get_node_parameters_interface(),
    node_.get_node_topics_interface(),
    false,
    tf2_ros::DynamicListenerQoS(),
    tf2_ros::StaticListenerQoS(),
    dynamic_options,
    static_options);

  pose_publisher_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
    config_.endpoints.pose_command_topic, rclcpp::SystemDefaultsQoS());
  command_type_client_ = node_.create_client<moveit_msgs::srv::ServoCommandType>(
    config_.endpoints.command_type_service,
    rclcpp::ServicesQoS(),
    callback_group_);
  pause_client_ = node_.create_client<std_srvs::srv::SetBool>(
    config_.endpoints.pause_service,
    rclcpp::ServicesQoS(),
    callback_group_);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  status_subscription_ = node_.create_subscription<moveit_msgs::msg::ServoStatus>(
    config_.endpoints.status_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](moveit_msgs::msg::ServoStatus::ConstSharedPtr message) {
      observe_status(std::move(message));
    },
    subscription_options);
  joint_state_subscription_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    config_.endpoints.joint_state_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
      observe_joint_state(std::move(message));
    },
    subscription_options);

  reset_ros_time_epoch();
}

MoveItServoPublishResult MoveItServoRosAdapter::Impl::publish_target()
{
  const std::lock_guard<std::mutex> target_lock(target_mutex_);
  if (!prepared_target_) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kInvalidData, "no prepared Servo target is available"),
      std::nullopt};
  }

  rclcpp::Time now{0, 0, node_.get_clock()->get_clock_type()};
  bool time_discontinuity = false;
  {
    const std::lock_guard<std::mutex> lock(feedback_mutex_);
    now = node_.now();
    observe_ros_time_locked(now);
    time_discontinuity = ros_time_moved_backwards_;
  }
  if (time_discontinuity) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kTimeDiscontinuity,
        "ROS time moved backwards during the active Servo target"),
      std::nullopt};
  }

  auto published = prepared_target_->servo_tip_goal;
  published.header.stamp = static_cast<builtin_interfaces::msg::Time>(now);
  if (!manipulation::validation::is_normalized_nonzero_stamp(published.header.stamp)) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kUnavailable,
        "node ROS clock has not produced a non-zero command stamp"),
      std::nullopt};
  }

  try {
    pose_publisher_->publish(published);
  } catch (const std::exception & error) {
    return {
      MoveItServoRosResult::failure(
        MoveItServoRosError::kUnavailable,
        std::string{"failed to publish Servo pose target: "} + error.what()),
      std::nullopt};
  }
  return {
    MoveItServoRosResult::success("published canonical Servo tip target"),
    std::move(published)};
}

const MoveItServoRosAdapterConfig & MoveItServoRosAdapter::Impl::config() const noexcept
{
  return config_;
}

MoveItServoRosAdapter::MoveItServoRosAdapter(
  rclcpp::Node & node, MoveItServoRosAdapterConfig config)
: impl_(std::make_unique<Impl>(node, std::move(config)))
{
}

MoveItServoRosAdapter::~MoveItServoRosAdapter() = default;

bool MoveItServoRosAdapter::backend_ready() const noexcept
{
  return impl_->backend_ready();
}

MoveItServoPrepareTargetResult MoveItServoRosAdapter::prepare_target(
  const geometry_msgs::msg::PoseStamped & task_goal,
  const Deadline deadline)
{
  return impl_->prepare_target(task_goal, deadline);
}

MoveItServoRosResult MoveItServoRosAdapter::configure_pose_mode(const Deadline deadline)
{
  return impl_->configure_pose_mode(deadline);
}

MoveItServoRosResult MoveItServoRosAdapter::unpause(const Deadline deadline)
{
  return impl_->set_paused(false, deadline);
}

MoveItServoPublishResult MoveItServoRosAdapter::publish_target()
{
  return impl_->publish_target();
}

MoveItServoObservationResult MoveItServoRosAdapter::latest_observation(
  const Deadline deadline)
{
  return impl_->latest_observation(deadline);
}

MoveItServoRosResult MoveItServoRosAdapter::pause(const Deadline deadline)
{
  return impl_->set_paused(true, deadline);
}

const MoveItServoRosAdapterConfig & MoveItServoRosAdapter::config() const noexcept
{
  return impl_->config();
}

}  // namespace manipulation::moveit
