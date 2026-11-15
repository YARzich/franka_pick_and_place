// Responsibility: Adapt TransferObjectSequenceV2 to the shared unlimited lifecycle core.

#include "manipulation/workflow/transfer_object_sequence_v2_action_server.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "detail/transfer_sequence_action_lifecycle_core.hpp"
#include "detail/transfer_sequence_goal_transport.hpp"
#include "manipulation/workflow/task_message_adapter.hpp"

namespace manipulation::workflow
{
namespace
{

using Core = detail::TransferSequenceActionLifecycleCore;
using GoalDescriptor = detail::TransferSequenceGoalDescriptor;

constexpr auto kLifecycleHeartbeat = std::chrono::milliseconds{100};

bool has_non_whitespace(const std::string & value)
{
  return value.find_first_not_of(" \t\r\n") != std::string::npos;
}

void validate_config(
  const TransferObjectSequenceV2ActionServerConfig & config,
  const TransferObjectSequenceV2BackendCallbacks & backend)
{
  if (!has_non_whitespace(config.action_name) ||
    !has_non_whitespace(config.pause_service_name))
  {
    throw std::invalid_argument("V2 transfer action and pause endpoints must not be empty");
  }
  if (config.action_name == config.pause_service_name) {
    throw std::invalid_argument("V2 transfer action and pause endpoints must differ");
  }
  if (!backend.admit || !backend.execute) {
    throw std::invalid_argument("V2 transfer admission and execute callbacks are required");
  }
}

std::optional<GoalDescriptor> describe_goal(
  const TransferObjectSequenceV2::Goal & goal)
{
  if (goal.steps.empty() ||
    goal.steps.size() > std::numeric_limits<std::uint32_t>::max())
  {
    return std::nullopt;
  }
  GoalDescriptor descriptor;
  descriptor.step_count = goal.steps.size();
  return descriptor;
}

std::uint8_t waiting_reason_value(const TransferWaitingReason reason) noexcept
{
  return static_cast<std::uint8_t>(reason);
}

detail::SequenceGoalTransportPort make_transport(
  const std::shared_ptr<
    rclcpp_action::ServerGoalHandle<TransferObjectSequenceV2>> & goal_handle)
{
  using TerminalKind = detail::SequenceTerminalKind;
  using TransportState = detail::SequenceGoalTransportState;
  return detail::SequenceGoalTransportPort(
    goal_handle->get_goal_id(),
    [goal_handle]() {
      if (goal_handle->is_canceling()) {
        return TransportState::kCanceling;
      }
      return goal_handle->is_executing() ?
             TransportState::kExecuting : TransportState::kOther;
    },
    [goal_handle](const TransferSequenceFeedbackSnapshot & state) {
      auto feedback = std::make_shared<TransferObjectSequenceV2::Feedback>();
      feedback->sequence_step_index = static_cast<std::uint32_t>(state.step_index);
      feedback->sequence_step_count = static_cast<std::uint32_t>(state.step_count);
      feedback->overall_progress = state.overall_progress;
      feedback->active_object_id = state.active_object_id;
      feedback->current_state = to_ros_message(state.current_state);
      feedback->waiting_reason = waiting_reason_value(state.waiting_reason);
      goal_handle->publish_feedback(feedback);
    },
    [goal_handle](
      const TerminalKind kind,
      const TransferSequenceExecution & execution,
      const std::chrono::steady_clock::duration elapsed) {
      auto result = std::make_shared<TransferObjectSequenceV2::Result>();
      result->result = to_ros_message(execution.result.outcome, elapsed);
      result->completed_step_count =
      static_cast<std::uint32_t>(execution.result.completed_step_count);
      result->failed_step_index = execution.result.failed_step_index ?
      static_cast<std::int64_t>(*execution.result.failed_step_index) :
      TransferObjectSequenceV2::Result::NO_FAILED_STEP;
      result->step_results.reserve(execution.result.step_outcomes.size());
      for (std::size_t index = 0U;
      index < execution.result.step_outcomes.size(); ++index)
      {
        result->step_results.push_back(
          to_ros_message(
            execution.result.step_outcomes[index], execution.step_elapsed[index]));
      }

      switch (kind) {
        case TerminalKind::kSucceeded:
          goal_handle->succeed(result);
          return;
        case TerminalKind::kCanceled:
          goal_handle->canceled(result);
          return;
        case TerminalKind::kAborted:
          goal_handle->abort(result);
          return;
      }
      throw std::logic_error("unknown V2 transfer terminal kind");
    });
}

TransferSequenceLifecycleConfig lifecycle_config()
{
  TransferSequenceLifecycleConfig config;
  config.heartbeat_period = kLifecycleHeartbeat;
  config.default_timeout = std::chrono::steady_clock::duration::max();
  return config;
}

}  // namespace

class TransferObjectSequenceV2ActionServer::Impl final
{
public:
  Impl(
    rclcpp::Node & node,
    const TransferObjectSequenceV2ActionServerConfig & config,
    TransferObjectSequenceV2BackendCallbacks backend)
  : backend_(std::make_shared<const TransferObjectSequenceV2BackendCallbacks>(
        std::move(backend))),
    callback_group_(
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    core_(std::make_unique<Core>(
        node, lifecycle_config(), "TransferObjectSequenceV2",
        config.pause_service_name,
        [backend = backend_](const TaskStopCause cause) {
          if (backend->request_stop) {
            backend->request_stop(cause);
          }
        },
        callback_group_, std::make_shared<TransferSequenceGoalReservation>()))
  {
    action_server_ = rclcpp_action::create_server<Action>(
      &node, config.action_name,
      [this](
        const rclcpp_action::GoalUUID & goal_id,
        const std::shared_ptr<const Action::Goal> goal) {
        const auto backend = backend_;
        return core_->handle_goal(
          goal_id, describe_goal(*goal),
          [goal, backend]() {return backend->admit(*goal);},
          [goal, backend](TransferSequenceTaskContext & context) {
            return backend->execute(*goal, context);
          });
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        return core_->handle_cancel(goal_handle->get_goal_id());
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        core_->handle_accepted(make_transport(goal_handle));
      },
      rcl_action_server_get_default_options(), callback_group_);
  }

  ~Impl()
  {
    core_.reset();
    action_server_.reset();
  }

  [[nodiscard]] TransferSequenceActionServerSnapshot snapshot() const
  {
    return core_->snapshot();
  }

private:
  std::shared_ptr<const TransferObjectSequenceV2BackendCallbacks> backend_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::unique_ptr<Core> core_;
};

TransferObjectSequenceV2ActionServer::TransferObjectSequenceV2ActionServer(
  rclcpp::Node & node,
  TransferObjectSequenceV2ActionServerConfig config,
  TransferObjectSequenceV2BackendCallbacks backend)
{
  validate_config(config, backend);
  impl_ = std::make_unique<Impl>(node, config, std::move(backend));
}

TransferObjectSequenceV2ActionServer::~TransferObjectSequenceV2ActionServer() = default;

TransferSequenceActionServerSnapshot
TransferObjectSequenceV2ActionServer::snapshot() const
{
  return impl_->snapshot();
}

}  // namespace manipulation::workflow
