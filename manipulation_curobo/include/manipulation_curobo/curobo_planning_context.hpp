// Responsibility: Owns one MoveIt planning request, its worker process and cancellation lifecycle.

#ifndef MANIPULATION_CUROBO__CUROBO_PLANNING_CONTEXT_HPP_
#define MANIPULATION_CUROBO__CUROBO_PLANNING_CONTEXT_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/logger.hpp>

#include "manipulation_curobo/child_process.hpp"
#include "manipulation_curobo/planning_admission.hpp"

namespace manipulation_curobo
{

/**
 * One MoveIt planning request executed by the external cuRobo bridge.
 *
 * PlanningScene and RobotModel are borrowed through MoveIt's context contract.
 * The context owns request-local cancellation and its exact child process
 * group, while a manager-owned gate bounds aggregate backend concurrency.
 * solve() runs on a planning worker while terminate() may run concurrently;
 * the solve thread's interruptible wait remains the only child-reaping path.
 */
class CuRoboPlanningContext final : public planning_interface::PlanningContext
{
public:
  /** Construct one request-local context with immutable backend resources. */
  CuRoboPlanningContext(
    const std::string & name, const std::string & group,
    const moveit::core::RobotModelConstPtr & model,
    std::string python_executable, std::string planner_script,
    std::string motion_gen_config, std::string workspace_frame,
    std::string curobo_ee_link,
    std::shared_ptr<curobo::PlanningAdmissionGate> admission_gate,
    const rclcpp::Logger & logger);

  /** Plan one request and return the standard MoveIt response. */
  void solve(planning_interface::MotionPlanResponse & response) override;

  /** Plan one request and return MoveIt's detailed response representation. */
  void solve(planning_interface::MotionPlanDetailedResponse & response) override;

  /// Request non-blocking cancellation of this context's exact child group.
  bool terminate() override;

  /// Request cancellation; the solve thread retains child-reaping ownership.
  void clear() override;

private:
  enum class PlanningOutcome
  {
    kSuccess,
    kPlanningFailed,
    kCanceled,
    kBusy,
    kTimedOut,
    kInvalidStartState,
    kInvalidRequest,
    kUnsupportedScene,
    kPlannerUnavailable,
    kInternalError,
  };

  [[nodiscard]] PlanningOutcome runPlanner(
    robot_trajectory::RobotTrajectory & trajectory,
    double & planning_time) const;

  /// Populate MoveIt's stable error code plus a backend-specific diagnostic.
  static void setErrorCode(
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    PlanningOutcome outcome);

  /// Forget only the process installed by the matching solve() invocation.
  void releaseActiveProcess(
    const std::shared_ptr<curobo::ChildProcess> & process) const;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * joint_model_group_{nullptr};
  std::string python_executable_;
  std::string planner_script_;
  std::string motion_gen_config_;
  std::string workspace_frame_;
  std::string curobo_ee_link_;
  std::shared_ptr<curobo::PlanningAdmissionGate> admission_gate_;
  rclcpp::Logger logger_;

  mutable std::mutex process_mutex_;
  mutable std::shared_ptr<curobo::ChildProcess> active_process_;
  mutable std::atomic_bool cancellation_requested_{false};
};

}  // namespace manipulation_curobo

#endif  // MANIPULATION_CUROBO__CUROBO_PLANNING_CONTEXT_HPP_
