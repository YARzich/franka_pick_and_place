// Responsibility: Validates neutral lifecycle settings and owns pause/heartbeat entities.

#include "detail/transfer_sequence_action_lifecycle_core.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "detail/transfer_sequence_action_active_goal.hpp"

namespace manipulation::workflow::detail
{
namespace
{

bool has_non_whitespace(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") != std::string::npos;
}

void validate_config(
  const TransferSequenceLifecycleConfig & config,
  const std::string & action_label,
  const std::string & pause_service_name,
  const rclcpp::CallbackGroup::SharedPtr & callback_group,
  const std::shared_ptr<TransferSequenceGoalReservation> & goal_reservation)
{
  using Duration = TransferSequenceLifecycleConfig::Clock::duration;

  if (!has_non_whitespace(action_label)) {
    throw std::invalid_argument("ordered-transfer action label must not be empty");
  }
  if (!has_non_whitespace(pause_service_name)) {
    throw std::invalid_argument(
            "ordered-transfer pause service name must not be empty");
  }
  if (!callback_group) {
    throw std::invalid_argument("ordered-transfer callback group must not be null");
  }
  if (!goal_reservation) {
    throw std::invalid_argument("ordered-transfer goal reservation must not be null");
  }
  if (config.heartbeat_period <= Duration::zero()) {
    throw std::invalid_argument("ordered-transfer heartbeat period must be positive");
  }
  if (config.default_timeout <= Duration::zero()) {
    throw std::invalid_argument("ordered-transfer default timeout must be positive");
  }
  if (config.accepted_start_delay < Duration::zero()) {
    throw std::invalid_argument(
            "ordered-transfer accepted-start delay must be non-negative");
  }
}

}  // namespace

TransferSequenceActionLifecycleCore::TransferSequenceActionLifecycleCore(
  rclcpp::Node & node,
  TransferSequenceLifecycleConfig config,
  std::string action_label,
  std::string pause_service_name,
  TransferSequenceRequestStopCallback request_stop,
  rclcpp::CallbackGroup::SharedPtr callback_group,
  std::shared_ptr<TransferSequenceGoalReservation> goal_reservation)
: config_(std::move(config)),
  action_label_(std::move(action_label)),
  request_stop_(std::move(request_stop)),
  logger_(node.get_logger()),
  goal_reservation_(std::move(goal_reservation)),
  callback_group_(std::move(callback_group))
{
  validate_config(
    config_, action_label_, pause_service_name, callback_group_, goal_reservation_);
  using std::placeholders::_1;
  using std::placeholders::_2;

  pause_service_ = node.create_service<PauseService>(
    pause_service_name,
    std::bind(&TransferSequenceActionLifecycleCore::handle_pause_request, this, _1, _2),
    rclcpp::ServicesQoS(), callback_group_);
  heartbeat_timer_ = node.create_wall_timer(
    config_.heartbeat_period,
    std::bind(&TransferSequenceActionLifecycleCore::heartbeat, this),
    callback_group_);
}

TransferSequenceActionLifecycleCore::~TransferSequenceActionLifecycleCore()
{
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
  }

  std::shared_ptr<ActiveGoal> active;
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    active = active_goal_;
  }
  if (active) {
    (void)set_stop_cause_if_none(active, TaskStopCause::kShutdown);
    (void)worker_.request_cancel();
    dispatch_backend_stop(active);
  } else {
    (void)worker_.request_cancel();
  }
}

}  // namespace manipulation::workflow::detail
