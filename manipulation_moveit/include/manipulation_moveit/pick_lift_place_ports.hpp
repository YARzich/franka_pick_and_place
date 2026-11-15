// Responsibility: Defines backend-neutral physical and coordinated-scene ports for pick/lift/place.

#ifndef MANIPULATION_MOVEIT__PICK_LIFT_PLACE_PORTS_HPP_
#define MANIPULATION_MOVEIT__PICK_LIFT_PLACE_PORTS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation/workflow/transfer_scene_workflow_port.hpp"
#include "manipulation/workflow/transfer_workflow_types.hpp"

namespace manipulation::moveit
{

/** Cheap, read-only readiness evidence used by the production health gate. */
struct PickLiftPlaceReadiness
{
  bool ready{false};
  std::string detail{"pick/lift/place readiness was not checked"};
};

/** Terminal result of the read-only production-readiness gate. */
enum class PickLiftPlaceHealthOutcome : std::uint8_t
{
  kReady,
  kUnavailable,
  kCanceled,
  kShutdown,
  kInternalError,
};

class PickLiftPlaceHealthResult final
{
public:
  [[nodiscard]] static PickLiftPlaceHealthResult ready(std::string detail = {});
  [[nodiscard]] static PickLiftPlaceHealthResult failure(
    PickLiftPlaceHealthOutcome outcome,
    std::string detail);

  [[nodiscard]] bool is_ready() const noexcept;
  [[nodiscard]] PickLiftPlaceHealthOutcome outcome() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  PickLiftPlaceHealthResult(PickLiftPlaceHealthOutcome outcome, std::string detail);

  PickLiftPlaceHealthOutcome outcome_{PickLiftPlaceHealthOutcome::kInternalError};
  std::string detail_{"pick/lift/place health result was not initialized"};
};

/**
 * Read-only goal-scoped readiness boundary for all production dependencies.
 *
 * This wait deliberately has no strategy timeout or polling parameter. It ends
 * only when dependencies are ready, a final health error is known, or the goal
 * is canceled/shut down through the context/stop request.
 */
class IPickLiftPlaceHealthPort
{
public:
  virtual ~IPickLiftPlaceHealthPort() = default;
  virtual void begin_goal() = 0;
  [[nodiscard]] virtual PickLiftPlaceHealthResult wait_until_ready(
    manipulation::workflow::TransferExecutionContext & context) = 0;
  virtual void request_stop() noexcept = 0;
};

/** Terminal status of one controller-backed physical command. */
enum class PhysicalCommandOutcome : std::uint8_t
{
  kSuccess,
  kRejected,
  kUnavailable,
  kFailed,
  kTimedOut,
  kCanceled,
  kShutdown,
  kInternalError,
};

/** Evidence determining whether the shared command resource may be released. */
enum class PhysicalCommandSafety : std::uint8_t
{
  /** The controller explicitly rejected the command before accepting ownership. */
  kNotAccepted,
  /** The accepted command reached a terminal result or a confirmed safe stop. */
  kTerminalSafe,
  /** An accepted command may still be active; ownership must remain quarantined. */
  kActiveStateUnknown,
};

/** Validated physical result shared by arm and gripper controller ports. */
class PhysicalCommandResult final
{
public:
  [[nodiscard]] static PhysicalCommandResult success(std::string detail = {});
  [[nodiscard]] static PhysicalCommandResult failure(
    PhysicalCommandOutcome outcome,
    PhysicalCommandSafety safety,
    std::string detail);

  [[nodiscard]] bool succeeded() const noexcept;
  [[nodiscard]] PhysicalCommandOutcome outcome() const noexcept;
  [[nodiscard]] PhysicalCommandSafety safety() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  PhysicalCommandResult(
    PhysicalCommandOutcome outcome,
    PhysicalCommandSafety safety,
    std::string detail);

  PhysicalCommandOutcome outcome_{PhysicalCommandOutcome::kInternalError};
  PhysicalCommandSafety safety_{PhysicalCommandSafety::kActiveStateUnknown};
  std::string detail_{"physical command result was not initialized"};
};

struct TrajectoryExecutionRequest
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::string label;
};

/**
 * Physical trajectory execution boundary, deliberately separate from planning.
 *
 * `accepted_motion_timeout` starts only after the controller accepts the goal.
 * Endpoint discovery and goal acceptance remain non-physical waits terminated
 * only by a final error, cancel, or shutdown. A timeout must report whether a
 * safe stop was confirmed or the command resource requires quarantine.
 */
class ITrajectoryExecutionPort
{
public:
  virtual ~ITrajectoryExecutionPort() = default;
  virtual void begin_goal() = 0;
  [[nodiscard]] virtual PhysicalCommandResult execute(
    const TrajectoryExecutionRequest & request,
    std::chrono::steady_clock::duration accepted_motion_timeout,
    manipulation::workflow::TransferExecutionContext & context) = 0;
  virtual void request_stop() noexcept = 0;
};

enum class GripperCommandKind : std::uint8_t
{
  kOpen,
  kClose,
};

enum class GripperPostcondition : std::uint8_t
{
  kNone,
  kOpenConfirmed,
  kClosedConfirmed,
  kUnknown,
};

struct StrategyGripperCommandRequest
{
  GripperCommandKind kind{GripperCommandKind::kOpen};
  double width_m{0.0};
  double max_effort_n{0.0};
  bool allow_stall{false};
  std::string label;
};

/** Physical result plus explicit gripper postcondition evidence. */
class StrategyGripperCommandResult final
{
public:
  [[nodiscard]] static StrategyGripperCommandResult success(
    GripperPostcondition postcondition,
    std::string detail = {});
  [[nodiscard]] static StrategyGripperCommandResult failure(
    PhysicalCommandOutcome outcome,
    PhysicalCommandSafety safety,
    GripperPostcondition postcondition,
    std::string detail);

  [[nodiscard]] const PhysicalCommandResult & command() const noexcept;
  [[nodiscard]] GripperPostcondition postcondition() const noexcept;

private:
  StrategyGripperCommandResult(
    PhysicalCommandResult command,
    GripperPostcondition postcondition);

  PhysicalCommandResult command_;
  GripperPostcondition postcondition_{GripperPostcondition::kUnknown};
};

/**
 * Controller-backed gripper boundary with a post-acceptance physical timeout.
 *
 * The timeout clock has the same acceptance boundary and evidence requirements
 * as `ITrajectoryExecutionPort`.
 */
class IStrategyGripperCommandPort
{
public:
  virtual ~IStrategyGripperCommandPort() = default;
  virtual void begin_goal() = 0;
  [[nodiscard]] virtual StrategyGripperCommandResult command(
    const StrategyGripperCommandRequest & request,
    std::chrono::steady_clock::duration accepted_command_timeout,
    manipulation::workflow::TransferExecutionContext & context) = 0;
  virtual void request_stop() noexcept = 0;
};

enum class CoordinatedSceneOutcome : std::uint8_t
{
  kSuccess,
  kConflict,
  kUnavailable,
  kFailed,
  kCanceled,
  kShutdown,
  kStateUncertain,
  kInternalError,
};

/** Result of a coordinator read/mutation, including the next full-scene lease. */
class CoordinatedSceneResult final
{
public:
  [[nodiscard]] static CoordinatedSceneResult success(
    manipulation::workflow::PlanningSceneLease lease,
    std::string detail = {});
  [[nodiscard]] static CoordinatedSceneResult failure(
    CoordinatedSceneOutcome outcome,
    std::string detail);

  [[nodiscard]] bool succeeded() const noexcept;
  [[nodiscard]] CoordinatedSceneOutcome outcome() const noexcept;
  [[nodiscard]] const std::optional<manipulation::workflow::PlanningSceneLease> &
  lease() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  CoordinatedSceneResult(
    CoordinatedSceneOutcome outcome,
    std::optional<manipulation::workflow::PlanningSceneLease> lease,
    std::string detail);

  CoordinatedSceneOutcome outcome_{CoordinatedSceneOutcome::kInternalError};
  std::optional<manipulation::workflow::PlanningSceneLease> lease_;
  std::string detail_{"coordinated scene result was not initialized"};
};

struct StrategySceneRevalidationRequest
{
  manipulation::workflow::PlanningSceneSnapshot scene_snapshot;
  manipulation::workflow::PlanningSceneAccessProof access_proof;
};

struct StrategySceneAttachRequest
{
  manipulation::workflow::PlanningSceneSnapshot scene_snapshot;
  manipulation::workflow::PlanningSceneAccessProof access_proof;
  std::string object_id;
  std::string attachment_link;
  std::vector<std::string> touch_links;
};

struct StrategySceneDetachRequest
{
  manipulation::workflow::PlanningSceneSnapshot scene_snapshot;
  manipulation::workflow::PlanningSceneAccessProof access_proof;
  std::string object_id;
  std::string attachment_link;
  geometry_msgs::msg::PoseStamped target_object_pose;
};

/** Goal-scoped authoritative scene reads and coordinated attach/detach mutations. */
class IPickLiftPlaceScenePort
{
public:
  virtual ~IPickLiftPlaceScenePort() = default;
  virtual void begin_goal() = 0;
  [[nodiscard]] virtual CoordinatedSceneResult revalidate(
    const StrategySceneRevalidationRequest & request,
    manipulation::workflow::TransferExecutionContext & context) = 0;
  [[nodiscard]] virtual CoordinatedSceneResult attach(
    const StrategySceneAttachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) = 0;
  [[nodiscard]] virtual CoordinatedSceneResult detach(
    const StrategySceneDetachRequest & request,
    manipulation::workflow::TransferExecutionContext & context) = 0;
  virtual void request_stop() noexcept = 0;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__PICK_LIFT_PLACE_PORTS_HPP_
