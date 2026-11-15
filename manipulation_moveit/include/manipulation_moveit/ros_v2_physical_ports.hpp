// Responsibility: Declares production ROS action adapters for V2 arm and gripper commands.

#ifndef MANIPULATION_MOVEIT__ROS_V2_PHYSICAL_PORTS_HPP_
#define MANIPULATION_MOVEIT__ROS_V2_PHYSICAL_PORTS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/pick_lift_place_ports.hpp"

namespace manipulation::moveit
{

/** Deployment endpoints only; physical timing and evidence thresholds stay private. */
struct RosTrajectoryExecutionPortConfig
{
  std::string action_name{"/execute_trajectory"};
  std::string joint_state_topic{"/joint_states"};
};

void validateRosTrajectoryExecutionPortConfig(
  const RosTrajectoryExecutionPortConfig & config);

/**
 * ExecuteTrajectory adapter with acceptance-relative physical timing.
 *
 * Discovery and goal acceptance have no semantic timeout. The supplied motion
 * budget starts in the accepted goal-response callback and also bounds the
 * terminal JointState convergence check. Before dispatch and after a successful
 * action result, fresh consecutive JointState samples must prove waypoint
 * agreement and quiescence. Exact-goal cancellation and late callbacks remain
 * owned by a generation-safe active slot; an unconfirmed terminal state is
 * reported as `kActiveStateUnknown` and keeps readiness fail-closed.
 */
class RosTrajectoryExecutionPort final : public ITrajectoryExecutionPort
{
public:
  RosTrajectoryExecutionPort(
    rclcpp::Node & node,
    RosTrajectoryExecutionPortConfig config = {});
  ~RosTrajectoryExecutionPort() override;

  RosTrajectoryExecutionPort(const RosTrajectoryExecutionPort &) = delete;
  RosTrajectoryExecutionPort & operator=(const RosTrajectoryExecutionPort &) = delete;
  RosTrajectoryExecutionPort(RosTrajectoryExecutionPort &&) = delete;
  RosTrajectoryExecutionPort & operator=(RosTrajectoryExecutionPort &&) = delete;

  void begin_goal() noexcept override;
  [[nodiscard]] PickLiftPlaceReadiness cached_readiness() const;
  [[nodiscard]] PhysicalCommandResult execute(
    const TrajectoryExecutionRequest & request,
    std::chrono::steady_clock::duration accepted_motion_timeout,
    manipulation::workflow::TransferExecutionContext & context) override;
  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/** Deployment endpoint only; command timing and grasp policy stay elsewhere. */
struct RosStrategyGripperPortConfig
{
  std::string action_name;
};

void validateRosStrategyGripperPortConfig(
  const RosStrategyGripperPortConfig & config);

/** GripperCommand adapter with confirmed open/closed postcondition evidence. */
class RosStrategyGripperPort final : public IStrategyGripperCommandPort
{
public:
  RosStrategyGripperPort(rclcpp::Node & node, RosStrategyGripperPortConfig config);
  ~RosStrategyGripperPort() override;

  RosStrategyGripperPort(const RosStrategyGripperPort &) = delete;
  RosStrategyGripperPort & operator=(const RosStrategyGripperPort &) = delete;
  RosStrategyGripperPort(RosStrategyGripperPort &&) = delete;
  RosStrategyGripperPort & operator=(RosStrategyGripperPort &&) = delete;

  void begin_goal() noexcept override;
  [[nodiscard]] PickLiftPlaceReadiness cached_readiness() const;
  [[nodiscard]] StrategyGripperCommandResult command(
    const StrategyGripperCommandRequest & request,
    std::chrono::steady_clock::duration accepted_command_timeout,
    manipulation::workflow::TransferExecutionContext & context) override;
  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__ROS_V2_PHYSICAL_PORTS_HPP_
