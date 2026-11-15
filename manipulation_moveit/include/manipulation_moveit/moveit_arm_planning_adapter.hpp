// Responsibility: Adapts plan-only arm requests to MoveIt's MoveGroup action.

#ifndef MANIPULATION_MOVEIT__MOVEIT_ARM_PLANNING_ADAPTER_HPP_
#define MANIPULATION_MOVEIT__MOVEIT_ARM_PLANNING_ADAPTER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "manipulation_moveit/arm_planning_port.hpp"

namespace manipulation::moveit
{

/** Robot and MoveIt planning profile. It deliberately contains no execution endpoint. */
struct MoveItArmPlanningAdapterConfig
{
  std::string move_group_action{"/move_action"};
  std::string planning_group;
  std::string end_effector_link;
  std::string planning_pipeline;
  std::string planner_id;
  std::chrono::steady_clock::duration action_poll_period{
    std::chrono::milliseconds{5}};
  double velocity_scaling{0.2};
  double acceleration_scaling{0.2};
  double position_tolerance{0.005};
  double orientation_tolerance{0.02};
  double joint_tolerance{0.01};
};

/** Validate endpoint, robot profile, scaling, and tolerances before clients are created. */
void validateMoveItArmPlanningAdapterConfig(const MoveItArmPlanningAdapterConfig & config);

/**
 * Thin plan-only MoveGroup adapter.
 *
 * The MoveIt request receives a mandatory internal planning budget owned by the
 * adapter implementation. The caller deliberately cannot turn it into a task or
 * backend-profile timeout. After dispatch, this adapter waits for a terminal
 * MoveGroup result, cancellation, or ROS shutdown; it never executes the returned
 * trajectory and owns no gripper or PlanningScene mutation endpoint.
 */
class MoveItArmPlanningAdapter final : public IArmPlanningBackend
{
public:
  MoveItArmPlanningAdapter(rclcpp::Node & node, MoveItArmPlanningAdapterConfig config);
  ~MoveItArmPlanningAdapter() override;

  MoveItArmPlanningAdapter(const MoveItArmPlanningAdapter &) = delete;
  MoveItArmPlanningAdapter & operator=(const MoveItArmPlanningAdapter &) = delete;
  MoveItArmPlanningAdapter(MoveItArmPlanningAdapter &&) = delete;
  MoveItArmPlanningAdapter & operator=(MoveItArmPlanningAdapter &&) = delete;

  [[nodiscard]] ArmPlanningReadiness cached_readiness() const override;
  [[nodiscard]] ArmPlanningResult plan_once(
    const ArmPlanningRequest & request,
    const PlanningStopToken & stop_token) override;
  void request_stop() noexcept override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__MOVEIT_ARM_PLANNING_ADAPTER_HPP_
