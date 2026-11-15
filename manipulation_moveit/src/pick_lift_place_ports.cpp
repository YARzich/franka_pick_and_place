// Responsibility: Enforces typed physical-command and coordinated-scene result invariants.

#include "manipulation_moveit/pick_lift_place_ports.hpp"

#include <stdexcept>
#include <utility>

namespace manipulation::moveit
{

PickLiftPlaceHealthResult PickLiftPlaceHealthResult::ready(std::string detail)
{
  return PickLiftPlaceHealthResult(PickLiftPlaceHealthOutcome::kReady, std::move(detail));
}

PickLiftPlaceHealthResult PickLiftPlaceHealthResult::failure(
  const PickLiftPlaceHealthOutcome outcome,
  std::string detail)
{
  if (outcome == PickLiftPlaceHealthOutcome::kReady) {
    throw std::invalid_argument("failed pick/lift/place health result cannot carry kReady");
  }
  return PickLiftPlaceHealthResult(outcome, std::move(detail));
}

PickLiftPlaceHealthResult::PickLiftPlaceHealthResult(
  const PickLiftPlaceHealthOutcome outcome,
  std::string detail)
: outcome_(outcome), detail_(std::move(detail))
{
}

bool PickLiftPlaceHealthResult::is_ready() const noexcept
{
  return outcome_ == PickLiftPlaceHealthOutcome::kReady;
}

PickLiftPlaceHealthOutcome PickLiftPlaceHealthResult::outcome() const noexcept
{
  return outcome_;
}

const std::string & PickLiftPlaceHealthResult::detail() const noexcept
{
  return detail_;
}

PhysicalCommandResult PhysicalCommandResult::success(std::string detail)
{
  return PhysicalCommandResult(
    PhysicalCommandOutcome::kSuccess, PhysicalCommandSafety::kTerminalSafe,
    std::move(detail));
}

PhysicalCommandResult PhysicalCommandResult::failure(
  const PhysicalCommandOutcome outcome,
  const PhysicalCommandSafety safety,
  std::string detail)
{
  if (outcome == PhysicalCommandOutcome::kSuccess) {
    throw std::invalid_argument("physical command failure cannot carry kSuccess");
  }
  if (outcome == PhysicalCommandOutcome::kTimedOut &&
    safety == PhysicalCommandSafety::kNotAccepted)
  {
    throw std::invalid_argument(
            "physical timeout cannot occur before controller goal acceptance");
  }
  return PhysicalCommandResult(outcome, safety, std::move(detail));
}

PhysicalCommandResult::PhysicalCommandResult(
  const PhysicalCommandOutcome outcome,
  const PhysicalCommandSafety safety,
  std::string detail)
: outcome_(outcome), safety_(safety), detail_(std::move(detail))
{
}

bool PhysicalCommandResult::succeeded() const noexcept
{
  return outcome_ == PhysicalCommandOutcome::kSuccess;
}

PhysicalCommandOutcome PhysicalCommandResult::outcome() const noexcept
{
  return outcome_;
}

PhysicalCommandSafety PhysicalCommandResult::safety() const noexcept
{
  return safety_;
}

const std::string & PhysicalCommandResult::detail() const noexcept
{
  return detail_;
}

StrategyGripperCommandResult StrategyGripperCommandResult::success(
  const GripperPostcondition postcondition,
  std::string detail)
{
  if (postcondition != GripperPostcondition::kOpenConfirmed &&
    postcondition != GripperPostcondition::kClosedConfirmed)
  {
    throw std::invalid_argument(
            "successful gripper command requires a confirmed open or closed postcondition");
  }
  return StrategyGripperCommandResult(
    PhysicalCommandResult::success(std::move(detail)), postcondition);
}

StrategyGripperCommandResult StrategyGripperCommandResult::failure(
  const PhysicalCommandOutcome outcome,
  const PhysicalCommandSafety safety,
  const GripperPostcondition postcondition,
  std::string detail)
{
  if (postcondition == GripperPostcondition::kOpenConfirmed ||
    postcondition == GripperPostcondition::kClosedConfirmed)
  {
    throw std::invalid_argument(
            "failed gripper command cannot claim a confirmed postcondition");
  }
  return StrategyGripperCommandResult(
    PhysicalCommandResult::failure(outcome, safety, std::move(detail)), postcondition);
}

StrategyGripperCommandResult::StrategyGripperCommandResult(
  PhysicalCommandResult command,
  const GripperPostcondition postcondition)
: command_(std::move(command)), postcondition_(postcondition)
{
}

const PhysicalCommandResult & StrategyGripperCommandResult::command() const noexcept
{
  return command_;
}

GripperPostcondition StrategyGripperCommandResult::postcondition() const noexcept
{
  return postcondition_;
}

CoordinatedSceneResult CoordinatedSceneResult::success(
  manipulation::workflow::PlanningSceneLease lease,
  std::string detail)
{
  return CoordinatedSceneResult(
    CoordinatedSceneOutcome::kSuccess,
    std::optional<manipulation::workflow::PlanningSceneLease>{std::move(lease)},
    std::move(detail));
}

CoordinatedSceneResult CoordinatedSceneResult::failure(
  const CoordinatedSceneOutcome outcome,
  std::string detail)
{
  if (outcome == CoordinatedSceneOutcome::kSuccess) {
    throw std::invalid_argument("coordinated scene failure cannot carry kSuccess");
  }
  return CoordinatedSceneResult(outcome, std::nullopt, std::move(detail));
}

CoordinatedSceneResult::CoordinatedSceneResult(
  const CoordinatedSceneOutcome outcome,
  std::optional<manipulation::workflow::PlanningSceneLease> lease,
  std::string detail)
: outcome_(outcome), lease_(std::move(lease)), detail_(std::move(detail))
{
}

bool CoordinatedSceneResult::succeeded() const noexcept
{
  return outcome_ == CoordinatedSceneOutcome::kSuccess && lease_.has_value();
}

CoordinatedSceneOutcome CoordinatedSceneResult::outcome() const noexcept
{
  return outcome_;
}

const std::optional<manipulation::workflow::PlanningSceneLease> &
CoordinatedSceneResult::lease() const noexcept
{
  return lease_;
}

const std::string & CoordinatedSceneResult::detail() const noexcept
{
  return detail_;
}

}  // namespace manipulation::moveit
