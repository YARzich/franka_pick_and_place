// Responsibility: Serve plan-only MoveGroup requests against an exact request-owned scene.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "moveit/move_group/capability_names.hpp"
#include "moveit/move_group/move_group_capability.hpp"
#include "moveit/moveit_cpp/moveit_cpp.hpp"
#include "moveit/planning_pipeline/planning_pipeline.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "exact_planning_scene.hpp"
#include "goal_planning_cancellation.hpp"

namespace manipulation::moveit
{
namespace
{

using MoveGroup = moveit_msgs::action::MoveGroup;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveGroup>;
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("manipulation_moveit.exact_scene_move_action");
}

void set_error(
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const std::int32_t value,
  std::string message)
{
  error_code.val = value;
  error_code.source = "manipulation_moveit/exact_scene_move_action";
  error_code.message = std::move(message);
}

class GoalAdmissionGuard final
{
public:
  explicit GoalAdmissionGuard(std::atomic_bool & goal_active) noexcept
  : goal_active_(goal_active)
  {
  }

  ~GoalAdmissionGuard()
  {
    goal_active_.store(false);
  }

  GoalAdmissionGuard(const GoalAdmissionGuard &) = delete;
  GoalAdmissionGuard & operator=(const GoalAdmissionGuard &) = delete;

private:
  std::atomic_bool & goal_active_;
};

}  // namespace

/**
 * Drop-in replacement for MoveGroupMoveAction on the canonical `move_action` endpoint.
 *
 * The upstream plan-only capability always clones its live PlanningSceneMonitor scene
 * and applies PlanningOptions::planning_scene_diff with diff semantics. This capability
 * deliberately accepts only plan-only goals carrying a full non-diff scene, constructs
 * an independent PlanningScene with setPlanningSceneMsg(), and invokes the selected
 * PlanningPipeline. A deployment must disable `move_group/MoveGroupMoveAction` before
 * enabling this capability so exactly one server owns the canonical endpoint.
 */
class ExactPlanningSceneMoveGroupCapability final : public move_group::MoveGroupCapability
{
public:
  ExactPlanningSceneMoveGroupCapability()
  : move_group::MoveGroupCapability("exact_planning_scene_move_action")
  {
  }

  ~ExactPlanningSceneMoveGroupCapability() override
  {
    shutdown_worker();
  }

  void initialize() override
  {
    const auto node = context_->moveit_cpp_->getNode();
    worker_ = std::thread([this]() {worker_loop();});
    try {
      action_server_ = rclcpp_action::create_server<MoveGroup>(
        node, move_group::MOVE_ACTION,
        [this](
          const rclcpp_action::GoalUUID & goal_id,
          const std::shared_ptr<const MoveGroup::Goal> goal)
        {
          if (!goal->planning_options.plan_only ||
          goal->planning_options.planning_scene_diff.is_diff ||
          goal->planning_options.planning_scene_diff.robot_state.is_diff)
          {
            RCLCPP_ERROR(
              logger(),
              "Exact-scene MoveGroup capability accepts only plan-only goals with a full scene");
            return rclcpp_action::GoalResponse::REJECT;
          }
          bool expected = false;
          if (!goal_active_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(logger(), "Rejecting concurrent exact-scene MoveGroup goal");
            return rclcpp_action::GoalResponse::REJECT;
          }
          {
            const std::lock_guard<std::mutex> lock(goal_mutex_);
            active_cancellation_ =
            std::make_shared<detail::GoalPlanningCancellation>(goal_id);
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> & goal_handle)
        {
          const auto cancellation = active_cancellation();
          return cancellation && cancellation->request(goal_handle->get_goal_id()) ?
                 rclcpp_action::CancelResponse::ACCEPT :
                 rclcpp_action::CancelResponse::REJECT;
        },
        [this](const std::shared_ptr<GoalHandle> goal_handle)
        {
          enqueue(goal_handle);
        });
    } catch (...) {
      shutdown_worker();
      throw;
    }
  }

private:
  void execute(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const GoalAdmissionGuard admission_guard(goal_active_);
    const auto cancellation = active_cancellation();
    auto result = std::make_shared<MoveGroup::Result>();
    try {
      auto feedback = std::make_shared<MoveGroup::Feedback>();
      feedback->state = stateToStr(move_group::PLANNING);
      goal_handle->publish_feedback(feedback);

      if (cancellation->requested()) {
        set_error(result->error_code, MoveItErrorCodes::PREEMPTED, "planning was canceled");
      } else {
        plan(goal_handle->get_goal(), *result, *cancellation);
      }
    } catch (const std::exception & exception) {
      set_error(
        result->error_code, MoveItErrorCodes::FAILURE,
        std::string{"exact-scene MoveGroup request failed: "} + exception.what());
    } catch (...) {
      set_error(
        result->error_code, MoveItErrorCodes::FAILURE,
        "exact-scene MoveGroup request failed with an unknown exception");
    }

    if (cancellation->commit_terminal()) {
      result->planned_trajectory = moveit_msgs::msg::RobotTrajectory{};
      result->trajectory_start = moveit_msgs::msg::RobotState{};
      set_error(result->error_code, MoveItErrorCodes::PREEMPTED, "planning was canceled");
    }

    try {
      const bool canceled =
        result->error_code.val == MoveItErrorCodes::PREEMPTED &&
        goal_handle->is_canceling();
      if (result->error_code.val == MoveItErrorCodes::SUCCESS) {
        goal_handle->succeed(result);
      } else if (canceled) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(logger(), "Cannot complete exact-scene MoveGroup goal: %s", exception.what());
    } catch (...) {
      RCLCPP_ERROR(logger(), "Cannot complete exact-scene MoveGroup goal: unknown exception");
    }
  }

  void plan(
    const std::shared_ptr<const MoveGroup::Goal> & goal,
    MoveGroup::Result & result,
    detail::GoalPlanningCancellation & cancellation)
  {
    const auto pipeline = resolvePlanningPipeline(goal->request.pipeline_id);
    if (!pipeline) {
      set_error(
        result.error_code, MoveItErrorCodes::FAILURE,
        "requested MoveIt planning pipeline is unavailable");
      return;
    }

    std::string scene_error;
    const auto scene = detail::make_exact_planning_scene(
      context_->planning_scene_monitor_->getRobotModel(),
      goal->planning_options.planning_scene_diff, scene_error);
    if (!scene) {
      set_error(
        result.error_code, MoveItErrorCodes::INVALID_ROBOT_STATE,
        std::move(scene_error));
      return;
    }

    planning_interface::MotionPlanResponse response;
    bool generated = false;
    const bool dispatched = cancellation.run(
      [&]() {
        generated = pipeline->generatePlan(scene, goal->request, response, context_->debug_);
      },
      [pipeline]() {pipeline->terminate();});
    if (!dispatched) {
      set_error(result.error_code, MoveItErrorCodes::PREEMPTED, "planning was canceled");
      return;
    }
    if (!generated && response.error_code.val == MoveItErrorCodes::SUCCESS) {
      set_error(
        response.error_code, MoveItErrorCodes::FAILURE,
        "MoveIt planning pipeline returned failure without an error code");
    }

    result.error_code = response.error_code;
    result.planning_time = response.planning_time;
    if (result.error_code.val != MoveItErrorCodes::SUCCESS) {
      return;
    }
    if (!response.trajectory) {
      set_error(
        result.error_code, MoveItErrorCodes::FAILURE,
        "MoveIt planning pipeline returned success without a trajectory");
      return;
    }
    convertToMsg(response.trajectory, result.trajectory_start, result.planned_trajectory);
  }

  void enqueue(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    bool queued = false;
    {
      const std::lock_guard<std::mutex> lock(worker_mutex_);
      if (!stop_worker_ && !pending_goal_) {
        pending_goal_ = goal_handle;
        queued = true;
      }
    }
    if (queued) {
      worker_cv_.notify_one();
      return;
    }

    goal_active_.store(false);
    auto result = std::make_shared<MoveGroup::Result>();
    set_error(
      result->error_code, MoveItErrorCodes::FAILURE,
      "exact-scene MoveGroup capability is shutting down");
    try {
      goal_handle->abort(result);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(logger(), "Cannot reject accepted MoveGroup goal: %s", exception.what());
    }
  }

  void worker_loop()
  {
    while (true) {
      std::shared_ptr<GoalHandle> goal_handle;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_cv_.wait(lock, [this]() {return stop_worker_ || pending_goal_;});
        if (stop_worker_ && !pending_goal_) {
          return;
        }
        goal_handle = std::move(pending_goal_);
      }
      try {
        execute(goal_handle);
      } catch (const std::exception & exception) {
        goal_active_.store(false);
        RCLCPP_ERROR(logger(), "Exact-scene MoveGroup worker failed: %s", exception.what());
      } catch (...) {
        goal_active_.store(false);
        RCLCPP_ERROR(logger(), "Exact-scene MoveGroup worker failed: unknown exception");
      }
    }
  }

  void shutdown_worker() noexcept
  {
    action_server_.reset();
    request_cancel();
    {
      const std::lock_guard<std::mutex> lock(worker_mutex_);
      stop_worker_ = true;
    }
    worker_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  std::shared_ptr<detail::GoalPlanningCancellation> active_cancellation()
  {
    const std::lock_guard<std::mutex> lock(goal_mutex_);
    return active_cancellation_;
  }

  void request_cancel() noexcept
  {
    if (const auto cancellation = active_cancellation()) {
      cancellation->request_shutdown();
    }
  }

  rclcpp_action::Server<MoveGroup>::SharedPtr action_server_;
  std::atomic_bool goal_active_{false};
  std::mutex goal_mutex_;
  std::shared_ptr<detail::GoalPlanningCancellation> active_cancellation_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::shared_ptr<GoalHandle> pending_goal_;
  bool stop_worker_{false};
  std::thread worker_;
};

}  // namespace manipulation::moveit

PLUGINLIB_EXPORT_CLASS(
  manipulation::moveit::ExactPlanningSceneMoveGroupCapability,
  move_group::MoveGroupCapability)
