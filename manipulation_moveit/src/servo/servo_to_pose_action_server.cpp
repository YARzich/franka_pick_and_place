// Responsibility: Validates ServoToPose action configuration and owns its ROS interfaces.

#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/expand_topic_or_service_name.hpp"

#include "manipulation/skill/standard_capabilities.hpp"
#include "servo_to_pose_active_goal.hpp"

namespace manipulation::moveit
{
namespace
{

void validate_endpoint(
  const std::string & endpoint, const char * label, const bool is_service)
{
  if (endpoint.empty()) {
    throw std::invalid_argument(std::string{label} + " must be non-empty");
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      endpoint, "servo_to_pose_action_server", "/", is_service);
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            std::string{label} + " is not a valid ROS name: " + error.what());
  }
}

void validate_config(const ServoToPoseActionServerConfig & config)
{
  using Duration = ServoToPoseActionServerConfig::Duration;
  validate_endpoint(config.action_name, "action_name", false);
  validate_endpoint(config.pause_service_name, "pause_service_name", true);
  if (config.heartbeat_period <= Duration::zero()) {
    throw std::invalid_argument("ServoToPose heartbeat_period must be positive");
  }
  if (config.default_timeout <= Duration::zero()) {
    throw std::invalid_argument("ServoToPose default_timeout must be positive");
  }
  if (const auto error = validate_servo_to_pose_task_config(config.task)) {
    throw std::invalid_argument("invalid ServoToPose task policy: " + *error);
  }
  if (!servo_to_pose_timeout_has_operational_budget(config.default_timeout, config.task)) {
    throw std::invalid_argument(
            "ServoToPose default_timeout must exceed halt_timeout plus one control period");
  }
  if (!config.skill_runtime) {
    throw std::invalid_argument(
            "ServoToPose requires a shared SkillRuntime from its production host");
  }

  const auto & descriptor = config.skill_runtime->descriptor();
  if (descriptor.resource_claims().empty()) {
    throw std::invalid_argument(
            "ServoToPose SkillRuntime must claim the host arm control resource");
  }
  if (!descriptor.required_capabilities().contains(
      manipulation::skill::capability::kServoPose) ||
    !descriptor.required_capabilities().contains(
      manipulation::skill::capability::kExactStampTransform))
  {
    throw std::invalid_argument(
            "ServoToPose SkillRuntime must require servo-pose and exact-stamp TF capabilities");
  }
}

}  // namespace

ServoToPoseActionServer::ServoToPoseActionServer(
  rclcpp::Node & node,
  ServoToPosePort & port,
  ServoToPoseActionServerConfig config)
: config_(std::move(config)),
  logger_(node.get_logger()),
  port_(port),
  task_(port, config_.task)
{
  validate_config(config_);
  using std::placeholders::_1;
  using std::placeholders::_2;

  callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_ = rclcpp_action::create_server<Action>(
    &node,
    config_.action_name,
    std::bind(&ServoToPoseActionServer::handle_goal, this, _1, _2),
    std::bind(&ServoToPoseActionServer::handle_cancel, this, _1),
    std::bind(&ServoToPoseActionServer::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    callback_group_);
  pause_service_ = node.create_service<PauseService>(
    config_.pause_service_name,
    std::bind(&ServoToPoseActionServer::handle_pause_request, this, _1, _2),
    rclcpp::ServicesQoS(),
    callback_group_);
  heartbeat_timer_ = node.create_wall_timer(
    config_.heartbeat_period,
    std::bind(&ServoToPoseActionServer::heartbeat, this),
    callback_group_);
}

ServoToPoseActionServer::~ServoToPoseActionServer()
{
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
  }
  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
  }
  if (active && active->control) {
    active->control->request_cancel();
  }
  if (worker_.joinable()) {
    if (worker_.get_id() == std::this_thread::get_id()) {
      std::terminate();
    }
    worker_.join();
  }
}

}  // namespace manipulation::moveit
