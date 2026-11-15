// Responsibility: Defines the plan-only arm boundary, typed outcomes, retry policy, and scene
//   revision gate used by transfer strategies.

#ifndef MANIPULATION_MOVEIT__ARM_PLANNING_PORT_HPP_
#define MANIPULATION_MOVEIT__ARM_PLANNING_PORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "manipulation/workflow/transfer_execution_context.hpp"

namespace manipulation::moveit
{

/** Immutable identity of the authoritative scene used to prepare a planning request. */
struct PlanningSceneRevision
{
  std::string session_id;
  std::uint64_t revision{0U};

  [[nodiscard]] bool operator==(const PlanningSceneRevision & other) const noexcept;
  [[nodiscard]] bool operator!=(const PlanningSceneRevision & other) const noexcept;
};

enum class ArmPlanningGoalKind : std::uint8_t
{
  kPose,
  kJoint,
};

/**
 * Backend-neutral arm goal plus its complete immutable authoritative scene input.
 *
 * `scene_snapshot` is value-owned and treated as immutable by the planning call.
 * It must be a full scene (`is_diff == false`), never a diff that depends on
 * hidden backend state. `scene_revision` identifies the source snapshot for
 * revalidation immediately before each attempt.
 */
struct ArmPlanningRequest
{
  ArmPlanningGoalKind goal_kind{ArmPlanningGoalKind::kPose};
  geometry_msgs::msg::PoseStamped target_pose;
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  std::string label;
  PlanningSceneRevision scene_revision;
  moveit_msgs::msg::PlanningScene scene_snapshot;
};

/** Stable terminal reason for one planning attempt or retry sequence. */
enum class PlanningOutcome : std::uint8_t
{
  kSuccess,
  kBudgetExhausted,
  kInvalidRequest,
  kInvalidStartState,
  kInvalidGoalState,
  kSceneRefreshRequired,
  kSceneUnavailable,
  kPlannerUnavailable,
  kPlanningFailed,
  kCanceled,
  kShutdown,
  kInternalError,
};

enum class PlanningFailureClass : std::uint8_t
{
  kNone,
  kRetryable,
  kNonRetryable,
  kStopped,
};

/** Map a typed outcome to the only control-flow classification used by retry policy. */
[[nodiscard]] PlanningFailureClass classifyPlanningOutcome(PlanningOutcome outcome) noexcept;

/** Plan-only result. It never reports controller or gripper execution evidence. */
class ArmPlanningResult final
{
public:
  [[nodiscard]] static ArmPlanningResult success(
    moveit_msgs::msg::RobotTrajectory trajectory,
    std::string detail = {});

  [[nodiscard]] static ArmPlanningResult failure(
    PlanningOutcome outcome,
    std::string detail,
    std::int32_t backend_error_code = 0);

  [[nodiscard]] PlanningOutcome outcome() const noexcept;
  [[nodiscard]] PlanningFailureClass failure_class() const noexcept;
  [[nodiscard]] bool succeeded() const noexcept;
  [[nodiscard]] bool retryable() const noexcept;
  [[nodiscard]] const moveit_msgs::msg::RobotTrajectory & trajectory() const noexcept;
  [[nodiscard]] std::int32_t backend_error_code() const noexcept;
  [[nodiscard]] std::size_t attempt_count() const noexcept;
  [[nodiscard]] const PlanningSceneRevision & planned_scene_revision() const noexcept;
  [[nodiscard]] const std::string & detail() const noexcept;

private:
  ArmPlanningResult(
    PlanningOutcome outcome,
    moveit_msgs::msg::RobotTrajectory trajectory,
    std::string detail,
    std::int32_t backend_error_code);

  void record_attempt_evidence(
    std::size_t attempt_count,
    PlanningSceneRevision scene_revision);

  PlanningOutcome outcome_{PlanningOutcome::kInternalError};
  moveit_msgs::msg::RobotTrajectory trajectory_;
  std::string detail_{"planning result was not initialized"};
  std::int32_t backend_error_code_{0};
  std::size_t attempt_count_{0U};
  PlanningSceneRevision planned_scene_revision_;

  friend class ArmPlanningPort;
};

enum class PlanningStopCause : std::uint8_t
{
  kNone,
  kCanceled,
  kShutdown,
};

namespace detail
{
struct PlanningStopState;
}  // namespace detail

/** Copyable request-local stop token observed by scene and planner adapters. */
class PlanningStopToken final
{
public:
  PlanningStopToken() = default;

  [[nodiscard]] bool stop_requested() const noexcept;
  [[nodiscard]] PlanningStopCause cause() const noexcept;
  [[nodiscard]] PlanningStopCause checkpoint() const;
  [[nodiscard]] PlanningStopCause heartbeat() const;
  [[nodiscard]] manipulation::workflow::TaskOperationGateResult run_operation(
    const std::function<void()> & operation) const;
  [[nodiscard]] PlanningStopCause wait_for(
    std::chrono::steady_clock::duration duration) const;

private:
  explicit PlanningStopToken(
    std::shared_ptr<detail::PlanningStopState> state,
    manipulation::workflow::TransferExecutionContext * execution_context = nullptr);

  std::shared_ptr<detail::PlanningStopState> state_;
  manipulation::workflow::TransferExecutionContext * execution_context_{nullptr};

  friend class ArmPlanningPort;
};

struct ArmPlanningReadiness
{
  bool ready{false};
  std::string detail{"planning readiness was not initialized"};
};

/** One backend attempt; implementations may plan but must never execute a trajectory. */
class IArmPlanningBackend
{
public:
  virtual ~IArmPlanningBackend() = default;
  [[nodiscard]] virtual ArmPlanningReadiness cached_readiness() const = 0;
  [[nodiscard]] virtual ArmPlanningResult plan_once(
    const ArmPlanningRequest & request,
    const PlanningStopToken & stop_token) = 0;
  virtual void request_stop() noexcept = 0;
};

enum class SceneRevisionState : std::uint8_t
{
  kCurrent,
  kRefreshRequired,
  kUnavailable,
};

struct SceneRevisionCheck
{
  SceneRevisionState state{SceneRevisionState::kUnavailable};
  PlanningSceneRevision observed;
  std::string detail{"scene revision was not checked"};
};

/** Read-only hook that proves the request still targets the authoritative scene revision. */
class IPlanningSceneRevisionHook
{
public:
  virtual ~IPlanningSceneRevisionHook() = default;
  [[nodiscard]] virtual SceneRevisionCheck revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken & stop_token) = 0;
};

/** Internal bounded retry schedule. It is constructor policy, not a ROS/YAML task setting. */
struct PlanningRetryPolicy
{
  std::size_t max_attempts{3U};
  std::chrono::steady_clock::duration initial_backoff{std::chrono::milliseconds{20}};
  std::chrono::steady_clock::duration maximum_backoff{std::chrono::milliseconds{100}};
};

/** Injectable wait boundary used to test cancellation at the backoff checkpoint. */
class IPlanningBackoff
{
public:
  virtual ~IPlanningBackoff() = default;
  [[nodiscard]] virtual PlanningStopCause wait(
    std::chrono::steady_clock::duration duration,
    const PlanningStopToken & stop_token) = 0;
};

/**
 * Plan-only port adding scene revision checks and a bounded retry policy to one backend.
 *
 * Exactly `kBudgetExhausted` is retried. Before every attempt the port revalidates
 * the immutable scene session/revision. A stale revision returns
 * `kSceneRefreshRequired` before another backend call, leaving refresh and all
 * physical side effects to the owning strategy. Cancel and shutdown interrupt
 * both backend work and backoff without starting a later attempt.
 */
class ArmPlanningPort final
{
public:
  ArmPlanningPort(
    IArmPlanningBackend & backend,
    IPlanningSceneRevisionHook & scene_revision_hook,
    PlanningRetryPolicy retry_policy = {});

  ArmPlanningPort(
    IArmPlanningBackend & backend,
    IPlanningSceneRevisionHook & scene_revision_hook,
    PlanningRetryPolicy retry_policy,
    IPlanningBackoff & backoff);

  ~ArmPlanningPort();

  ArmPlanningPort(const ArmPlanningPort &) = delete;
  ArmPlanningPort & operator=(const ArmPlanningPort &) = delete;
  ArmPlanningPort(ArmPlanningPort &&) = delete;
  ArmPlanningPort & operator=(ArmPlanningPort &&) = delete;

  void begin_goal();
  [[nodiscard]] ArmPlanningReadiness cached_readiness() const;
  [[nodiscard]] ArmPlanningResult plan(const ArmPlanningRequest & request);
  [[nodiscard]] ArmPlanningResult plan(
    const ArmPlanningRequest & request,
    manipulation::workflow::TransferExecutionContext & execution_context);
  void request_stop(PlanningStopCause cause) noexcept;

private:
  [[nodiscard]] ArmPlanningResult plan_impl(
    const ArmPlanningRequest & request,
    manipulation::workflow::TransferExecutionContext * execution_context);
  [[nodiscard]] std::shared_ptr<detail::PlanningStopState> current_stop_state() const;

  IArmPlanningBackend & backend_;
  IPlanningSceneRevisionHook & scene_revision_hook_;
  const PlanningRetryPolicy retry_policy_;
  std::unique_ptr<IPlanningBackoff> owned_backoff_;
  IPlanningBackoff * backoff_{nullptr};
  mutable std::mutex stop_state_mutex_;
  std::shared_ptr<detail::PlanningStopState> stop_state_;
  std::mutex plan_mutex_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__ARM_PLANNING_PORT_HPP_
