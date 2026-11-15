// Responsibility: Serializes a MoveIt request, supervises cuRobo execution and maps its result
//   back.

#include "manipulation_curobo/curobo_planning_context.hpp"

#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manipulation_curobo/temporary_file.hpp"
#include "planning_io.hpp"

namespace manipulation_curobo
{
namespace
{

constexpr double kStartPointTolerance = 1.0e-4;

const char * outcomeName(const curobo::ProcessOutcome outcome)
{
  switch (outcome) {
    case curobo::ProcessOutcome::kExited:
      return "exited";
    case curobo::ProcessOutcome::kSignaled:
      return "signaled";
    case curobo::ProcessOutcome::kTimedOut:
      return "timed_out";
    case curobo::ProcessOutcome::kCanceled:
      return "canceled";
    case curobo::ProcessOutcome::kSpawnFailed:
      return "spawn_failed";
    case curobo::ProcessOutcome::kWaitFailed:
      return "wait_failed";
  }
  return "unknown";
}

}  // namespace

CuRoboPlanningContext::CuRoboPlanningContext(
  const std::string & name,
  const std::string & group,
  const moveit::core::RobotModelConstPtr & model,
  std::string python_executable,
  std::string planner_script,
  std::string motion_gen_config,
  std::string workspace_frame,
  std::string curobo_ee_link,
  std::shared_ptr<curobo::PlanningAdmissionGate> admission_gate,
  const rclcpp::Logger & logger)
: planning_interface::PlanningContext(name, group),
  robot_model_(model),
  python_executable_(std::move(python_executable)),
  planner_script_(std::move(planner_script)),
  motion_gen_config_(std::move(motion_gen_config)),
  workspace_frame_(std::move(workspace_frame)),
  curobo_ee_link_(std::move(curobo_ee_link)),
  admission_gate_(std::move(admission_gate)),
  logger_(logger)
{
  if (robot_model_) {
    joint_model_group_ = robot_model_->getJointModelGroup(group);
  }
}

void CuRoboPlanningContext::solve(planning_interface::MotionPlanResponse & response)
{
  robot_trajectory::RobotTrajectory trajectory(robot_model_, getGroupName());
  double planning_time = 0.0;
  const PlanningOutcome outcome = runPlanner(trajectory, planning_time);
  setErrorCode(response.error_code, outcome);
  response.planning_time = planning_time;
  response.trajectory =
    std::make_shared<robot_trajectory::RobotTrajectory>(std::move(trajectory));
}

void CuRoboPlanningContext::solve(
  planning_interface::MotionPlanDetailedResponse & response)
{
  robot_trajectory::RobotTrajectory trajectory(robot_model_, getGroupName());
  double planning_time = 0.0;
  const PlanningOutcome outcome = runPlanner(trajectory, planning_time);
  setErrorCode(response.error_code, outcome);
  if (outcome != PlanningOutcome::kSuccess) {
    return;
  }
  response.description.push_back("cuRobo_plan");
  response.trajectory.push_back(
    std::make_shared<robot_trajectory::RobotTrajectory>(std::move(trajectory)));
  response.processing_time.push_back(planning_time);
}

bool CuRoboPlanningContext::terminate()
{
  cancellation_requested_.store(true);
  std::shared_ptr<curobo::ChildProcess> process;
  {
    std::lock_guard<std::mutex> lock(process_mutex_);
    process = active_process_;
  }
  if (process) {
    process->requestStop();
  }
  // Cancellation remains valid before solve() installs the physical child.
  return true;
}

void CuRoboPlanningContext::clear()
{
  (void)terminate();
}

CuRoboPlanningContext::PlanningOutcome CuRoboPlanningContext::runPlanner(
  robot_trajectory::RobotTrajectory & trajectory,
  double & planning_time) const
{
  const auto bridge_outcome = [](const curobo::io::BridgeResultCode outcome) {
      using Bridge = curobo::io::BridgeResultCode;
      switch (outcome) {
        case Bridge::kSuccess:
          return PlanningOutcome::kSuccess;
        case Bridge::kPlanningFailed:
          return PlanningOutcome::kPlanningFailed;
        case Bridge::kTimedOut:
          return PlanningOutcome::kTimedOut;
        case Bridge::kInvalidRequest:
          return PlanningOutcome::kInvalidRequest;
        case Bridge::kUnsupportedScene:
          return PlanningOutcome::kUnsupportedScene;
        case Bridge::kInternalError:
        case Bridge::kInvalidResponse:
          return PlanningOutcome::kInternalError;
      }
      return PlanningOutcome::kInternalError;
    };
  planning_time = 0.0;
  if (!joint_model_group_ || !robot_model_) {
    RCLCPP_ERROR(logger_, "cuRobo planning group is absent from RobotModel");
    return PlanningOutcome::kInternalError;
  }
  if (cancellation_requested_.load()) {
    RCLCPP_WARN(logger_, "cuRobo request was canceled before planner startup");
    return PlanningOutcome::kCanceled;
  }
  if (!admission_gate_) {
    RCLCPP_ERROR(logger_, "cuRobo planner admission gate is unavailable");
    return PlanningOutcome::kInternalError;
  }

  auto admission = admission_gate_->tryAcquire();
  if (!admission) {
    RCLCPP_WARN(
      logger_, "cuRobo planner BUSY: all %zu planning slot(s) are in use",
      admission_gate_->capacity());
    return PlanningOutcome::kBusy;
  }
  if (cancellation_requested_.load()) {
    RCLCPP_WARN(logger_, "cuRobo request was canceled during planner admission");
    return PlanningOutcome::kCanceled;
  }

  const auto & request = getMotionPlanRequest();
  const auto scene = getPlanningScene();
  std::string error;
  moveit::core::RobotState resolved_start_state(robot_model_);
  std::vector<double> start_positions;
  if (!curobo::io::extractStartState(
      request, *joint_model_group_, scene, resolved_start_state, start_positions, error))
  {
    RCLCPP_ERROR(logger_, "Invalid cuRobo start state: %s", error.c_str());
    return PlanningOutcome::kInvalidStartState;
  }

  curobo::io::GoalData goal;
  if (!curobo::io::extractGoal(
      request, robot_model_, *joint_model_group_, curobo_ee_link_, workspace_frame_,
      goal, error))
  {
    RCLCPP_ERROR(logger_, "Unsupported cuRobo goal: %s", error.c_str());
    return PlanningOutcome::kInvalidRequest;
  }

  try {
    curobo::TemporaryFile request_file{"curobo_request"};
    curobo::TemporaryFile response_file{"curobo_response"};
    const auto request_result = curobo::io::writePlanningRequest(
      request, scene, resolved_start_state, start_positions,
      joint_model_group_->getVariableNames(), goal, motion_gen_config_,
      workspace_frame_, request_file.path(), error);
    if (request_result != curobo::io::BridgeResultCode::kSuccess) {
      RCLCPP_ERROR(logger_, "Cannot serialize cuRobo request: %s", error.c_str());
      return bridge_outcome(request_result);
    }

    auto process = std::make_shared<curobo::ChildProcess>();
    {
      std::lock_guard<std::mutex> lock(process_mutex_);
      if (active_process_) {
        RCLCPP_ERROR(logger_, "Concurrent solve() calls on one cuRobo context are unsupported");
        return PlanningOutcome::kInternalError;
      }
      active_process_ = process;
      if (cancellation_requested_.load()) {
        process->requestStop();
      }
    }

    const std::vector<std::string> arguments{
      python_executable_, planner_script_, "--input", request_file.path().string(),
      "--output", response_file.path().string()};
    const bool started = process->start(arguments);
    // MotionGen import/configuration/warmup precedes its plan_single* call and
    // is deliberately outside allowed_planning_time. Python passes that budget
    // only to MotionGenPlanConfig; this supervisor wait is canceled explicitly.
    const auto process_result = process->wait();
    releaseActiveProcess(process);

    if (!started || !process_result.succeeded()) {
      if (process_result.outcome == curobo::ProcessOutcome::kCanceled) {
        RCLCPP_WARN(
          logger_, "cuRobo bridge outcome=%s exit_code=%d signal=%d: %s",
          outcomeName(process_result.outcome), process_result.exit_code,
          process_result.signal_number, process_result.detail.c_str());
      } else {
        RCLCPP_ERROR(
          logger_, "cuRobo bridge outcome=%s exit_code=%d signal=%d: %s",
          outcomeName(process_result.outcome), process_result.exit_code,
          process_result.signal_number, process_result.detail.c_str());
      }
      if (process_result.outcome == curobo::ProcessOutcome::kCanceled) {
        return PlanningOutcome::kCanceled;
      }
      if (process_result.outcome == curobo::ProcessOutcome::kTimedOut) {
        return PlanningOutcome::kTimedOut;
      }
      return PlanningOutcome::kPlannerUnavailable;
    }
    if (cancellation_requested_.load()) {
      RCLCPP_WARN(logger_, "Discarding a late cuRobo result after cancellation");
      return PlanningOutcome::kCanceled;
    }

    const auto response_result = curobo::io::loadPlanningResponse(
      response_file.path(), robot_model_, *joint_model_group_, resolved_start_state,
      scene, kStartPointTolerance, trajectory, planning_time, error);
    if (response_result != curobo::io::BridgeResultCode::kSuccess) {
      RCLCPP_ERROR(logger_, "Invalid cuRobo planning response: %s", error.c_str());
      return bridge_outcome(response_result);
    }
    if (cancellation_requested_.load()) {
      RCLCPP_WARN(logger_, "Discarding a parsed cuRobo trajectory after cancellation");
      trajectory.clear();
      return PlanningOutcome::kCanceled;
    }
    return PlanningOutcome::kSuccess;
  } catch (const std::exception & exception) {
    std::shared_ptr<curobo::ChildProcess> process;
    {
      std::lock_guard<std::mutex> lock(process_mutex_);
      process = active_process_;
    }
    if (process) {
      process->requestStop();
      (void)process->wait();
      releaseActiveProcess(process);
    }
    RCLCPP_ERROR(logger_, "cuRobo planner failed safely: %s", exception.what());
    return PlanningOutcome::kInternalError;
  }
}

void CuRoboPlanningContext::setErrorCode(
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const PlanningOutcome outcome)
{
  error_code.source = "manipulation_curobo";
  switch (outcome) {
    case PlanningOutcome::kSuccess:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      error_code.message.clear();
      return;
    case PlanningOutcome::kCanceled:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
      error_code.message = "cuRobo planning was canceled";
      return;
    case PlanningOutcome::kBusy:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      error_code.message = "cuRobo planner BUSY: configured planning capacity is in use";
      return;
    case PlanningOutcome::kPlanningFailed:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      error_code.message = "cuRobo could not find a collision-free plan";
      return;
    case PlanningOutcome::kTimedOut:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
      error_code.message = "cuRobo exhausted the request planning budget";
      return;
    case PlanningOutcome::kInvalidStartState:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      error_code.message = "cuRobo received an invalid start state";
      return;
    case PlanningOutcome::kInvalidRequest:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      error_code.message = "cuRobo received an invalid planning request";
      return;
    case PlanningOutcome::kUnsupportedScene:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE;
      error_code.message = "cuRobo cannot represent the supplied PlanningScene exactly";
      return;
    case PlanningOutcome::kPlannerUnavailable:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::COMMUNICATION_FAILURE;
      error_code.message = "cuRobo bridge process is unavailable";
      return;
    case PlanningOutcome::kInternalError:
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      error_code.message = "cuRobo returned an invalid or internal result";
      return;
  }
}

void CuRoboPlanningContext::releaseActiveProcess(
  const std::shared_ptr<curobo::ChildProcess> & process) const
{
  std::lock_guard<std::mutex> lock(process_mutex_);
  if (active_process_ == process) {
    active_process_.reset();
  }
}

}  // namespace manipulation_curobo
