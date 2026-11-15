// Responsibility: Serve ExecuteTrajectory with exact cancellation and one owned execution worker.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "moveit/move_group/capability_names.hpp"
#include "moveit/move_group/move_group_capability.hpp"
#include "moveit/moveit_cpp/moveit_cpp.hpp"
#include "moveit/trajectory_execution_manager/trajectory_execution_manager.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "cancelable_trajectory_execution.hpp"

namespace manipulation::moveit
{
namespace
{

using Action = moveit_msgs::action::ExecuteTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
using Status = detail::TrajectoryExecutionStatus;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("manipulation_moveit.cancelable_execute_trajectory");
}

Status execution_status(const moveit_controller_manager::ExecutionStatus & status)
{
  if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
    return Status::kSucceeded;
  }
  if (status == moveit_controller_manager::ExecutionStatus::PREEMPTED) {
    return Status::kCanceled;
  }
  if (status == moveit_controller_manager::ExecutionStatus::TIMED_OUT) {
    return Status::kTimedOut;
  }
  return Status::kFailed;
}

std::int32_t error_code(const Status status)
{
  switch (status) {
    case Status::kSucceeded:
      return ErrorCodes::SUCCESS;
    case Status::kCanceled:
      return ErrorCodes::PREEMPTED;
    case Status::kTimedOut:
      return ErrorCodes::TIMED_OUT;
    case Status::kFailed:
      return ErrorCodes::CONTROL_FAILED;
  }
  return ErrorCodes::FAILURE;
}

}  // namespace

/**
 * Replace upstream MoveGroupExecuteTrajectoryAction on its existing endpoint.
 * ROS callbacks only reserve/queue/cancel exact UUIDs. One worker owns all TEM
 * calls, including stop and join; terminal transport cannot precede that join.
 */
class CancelableExecuteTrajectoryCapability final : public move_group::MoveGroupCapability
{
public:
  CancelableExecuteTrajectoryCapability()
  : move_group::MoveGroupCapability("cancelable_execute_trajectory")
  {
  }

  ~CancelableExecuteTrajectoryCapability() override
  {
    shutting_down_.store(true);
    // Disable result transport before retiring unresolved handles: rclcpp's
    // unfinished GoalHandle destructor otherwise synthesizes a canceled result.
    action_server_.reset();
    execution_.shutdown();
  }

  void initialize() override
  {
    const auto node = context_->moveit_cpp_->getNode();
    ros_context_ = node->get_node_base_interface()->get_context();
    action_server_ = rclcpp_action::create_server<Action>(
      node, move_group::EXECUTE_ACTION_NAME,
      [this](const rclcpp_action::GoalUUID & id, std::shared_ptr<const Action::Goal>) {
        if (shutting_down_.load() || !context_->trajectory_execution_manager_ ||
        !execution_.reserve(id))
        {
          RCLCPP_WARN(logger(), "Rejecting unavailable or concurrent ExecuteTrajectory goal");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> goal) {
        if (!execution_.request_cancel(goal->get_goal_id())) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        RCLCPP_INFO(logger(), "Exact ExecuteTrajectory cancellation requested");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal) {enqueue(goal);});
  }

private:
  void enqueue(const std::shared_ptr<GoalHandle> & goal)
  {
    const auto manager = context_->trajectory_execution_manager_;
    detail::TrajectoryExecutionOperations operations;
    operations.push = [manager, goal]() {
        return manager->push(goal->get_goal()->trajectory, goal->get_goal()->controller_names);
      };
    operations.start = [manager, goal](const auto & completed) {
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->state = "MONITOR";
        goal->publish_feedback(feedback);
        manager->execute([completed](const auto & status) {completed(execution_status(status));});
      };
    operations.stop = [manager]() {manager->stopExecution(true);};
    operations.join = [manager]() {(void)manager->waitForExecution();};
    operations.finish = [this, goal](const detail::TrajectoryExecutionOutcome & outcome) {
        finish(goal, outcome);
      };
    operations.fault = [](const std::string & message) {
        RCLCPP_ERROR(
          logger(), "%s; retaining unresolved goal and rejecting new goals until restart",
          message.c_str());
      };
    if (!execution_.submit(goal->get_goal_id(), std::move(operations))) {
      // Rejected enqueue has not touched TEM. This can only occur during shutdown.
      finish(goal, {Status::kFailed, "ExecuteTrajectory capability is shutting down"});
    }
  }

  void finish(
    const std::shared_ptr<GoalHandle> & goal, const detail::TrajectoryExecutionOutcome & outcome)
  {
    auto result = std::make_shared<Action::Result>();
    result->error_code.val = error_code(outcome.status);
    result->error_code.source = "manipulation_moveit/cancelable_execute_trajectory";
    result->error_code.message = outcome.detail;
    // A cancel callback records intent before rclcpp transitions the handle to
    // CANCELING. Do not race that transition or wait for it during plugin shutdown.
    while (outcome.cancellation_requested && !goal->is_canceling() &&
      !shutting_down_.load() && ros_context_->is_valid())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (!ros_context_->is_valid()) {
      return;
    }
    if (outcome.status == Status::kSucceeded) {
      goal->succeed(result);
    } else if (goal->is_canceling()) {
      goal->canceled(result);
    } else {
      goal->abort(result);
    }
    RCLCPP_INFO(logger(), "ExecuteTrajectory terminal code=%d: %s",
      result->error_code.val, outcome.detail.c_str());
  }

  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::shared_ptr<rclcpp::Context> ros_context_;
  std::atomic_bool shutting_down_{false};
  detail::CancelableTrajectoryExecution execution_;
};

}  // namespace manipulation::moveit

PLUGINLIB_EXPORT_CLASS(
  manipulation::moveit::CancelableExecuteTrajectoryCapability,
  move_group::MoveGroupCapability)
