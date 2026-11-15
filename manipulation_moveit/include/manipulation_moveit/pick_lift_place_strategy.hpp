// Responsibility: Declares the direct C++ production pick/lift/place execution strategy.

#ifndef MANIPULATION_MOVEIT__PICK_LIFT_PLACE_STRATEGY_HPP_
#define MANIPULATION_MOVEIT__PICK_LIFT_PLACE_STRATEGY_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "manipulation/workflow/transfer_workflow_ports.hpp"

#include "manipulation_moveit/arm_planning_port.hpp"
#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace manipulation::moveit
{

/** Robot-specific binding and named targets; never inferred by the strategy. */
struct PickLiftPlaceRobotProfile
{
  std::string attachment_link;
  std::vector<std::string> touch_links;
  double open_width_m{std::numeric_limits<double>::quiet_NaN()};
  bool allow_grasp_stall{false};
  std::vector<std::string> home_joint_names;
  std::vector<double> home_joint_positions;
};

/**
 * Task-family stage geometry; the approach is owned by each grasp candidate.
 *
 * Clearance distances are applied along the reverse of the selected candidate's
 * approach direction, on both the pick and the place side.
 */
struct PickLiftPlaceTaskProfile
{
  double lift_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double preplace_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double retreat_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::duration release_settle{};
};

/** Profiles plus exactly the two accepted physical-command bounds. */
struct PickLiftPlaceStrategyConfig
{
  PickLiftPlaceRobotProfile robot;
  PickLiftPlaceTaskProfile task;

  /** Starts only after a trajectory controller accepts a physical goal. */
  std::chrono::steady_clock::duration arm_motion_timeout{};
  /** Starts only after a gripper controller accepts a physical goal. */
  std::chrono::steady_clock::duration gripper_command_timeout{};
};

void validatePickLiftPlaceStrategyConfig(const PickLiftPlaceStrategyConfig & config);

/**
 * Production `pick_lift_place` strategy called directly by the resolved workflow.
 *
 * Planning, physical execution, gripper commands, and scene mutations are
 * separate borrowed ports. The strategy owns no ROS action server and never
 * lets the planning adapter execute a trajectory. Every physical or scene side
 * effect is preceded by a cooperative checkpoint and exact lease/session/revision
 * revalidation. Dependencies must outlive the strategy and any active execute().
 */
class PickLiftPlaceStrategy final
  : public manipulation::workflow::TransferExecutionStrategy
{
public:
  PickLiftPlaceStrategy(
    PickLiftPlaceStrategyConfig config,
    ArmPlanningPort & planning,
    ITrajectoryExecutionPort & trajectory_execution,
    IStrategyGripperCommandPort & gripper,
    IPickLiftPlaceScenePort & scene,
    IPickLiftPlaceHealthPort & health);
  ~PickLiftPlaceStrategy() override;

  PickLiftPlaceStrategy(const PickLiftPlaceStrategy &) = delete;
  PickLiftPlaceStrategy & operator=(const PickLiftPlaceStrategy &) = delete;
  PickLiftPlaceStrategy(PickLiftPlaceStrategy &&) = delete;
  PickLiftPlaceStrategy & operator=(PickLiftPlaceStrategy &&) = delete;

  [[nodiscard]] std::string_view id() const noexcept override;
  [[nodiscard]] manipulation::task::TaskOutcome execute(
    const manipulation::workflow::ResolvedTransferTask & task,
    manipulation::workflow::TransferExecutionContext & context) override;
  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__PICK_LIFT_PLACE_STRATEGY_HPP_
