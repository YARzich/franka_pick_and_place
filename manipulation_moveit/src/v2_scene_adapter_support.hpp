// Responsibility: Declares pure request, state-bracket, and interruptible-wait logic for V2.

#ifndef V2_SCENE_ADAPTER_SUPPORT_HPP_
#define V2_SCENE_ADAPTER_SUPPORT_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"

#include "manipulation/workflow/object_catalog.hpp"
#include "manipulation_msgs/msg/planning_scene_coordinator_state.hpp"
#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"
#include "manipulation_msgs/srv/apply_planning_scene_diff.hpp"

#include "manipulation_moveit/arm_planning_port.hpp"

namespace manipulation::moveit::detail
{

/** Cancel epoch that a new goal explicitly resets without touching lease state. */
class GoalStopGeneration final
{
public:
  void begin_goal() noexcept;
  void request_stop() noexcept;
  [[nodiscard]] std::uint64_t capture() const noexcept;
  [[nodiscard]] bool stopped(std::uint64_t generation) const noexcept;

private:
  std::atomic<std::uint64_t> stop_generation_{0U};
  std::atomic<std::uint64_t> goal_generation_{0U};
};

enum class InterruptibleWaitResult : std::uint8_t
{
  kReady,
  kStopped,
  kShutdown,
  kFailed,
};

enum class SceneDispatchClass : std::uint8_t
{
  kGoalSideEffect,
  kLeaseMaintenance,
};

/** Route a short scene transport send through the correct lifecycle gate. */
[[nodiscard]] manipulation::workflow::TaskOperationGateResult dispatch_scene_operation(
  manipulation::workflow::TransferExecutionContext & context,
  SceneDispatchClass dispatch_class,
  const std::function<void()> & operation);

/** Pure unbounded wait loop; each wait slice is supplied by the owning context/token. */
[[nodiscard]] InterruptibleWaitResult wait_for_scene_condition(
  const std::function<bool()> & ready,
  const std::function<InterruptibleWaitResult()> & wait_one_slice,
  const std::function<bool()> & ros_context_valid);

/** Canonical sorted union of world, attached, and object/touch-link ACM resources. */
[[nodiscard]] std::vector<std::string> make_goal_scene_resource_keys(
  const std::vector<manipulation::workflow::ObjectInstance> & instances,
  const std::vector<std::string> & touch_links);

/** Explicit full-scene request, including robot state, geometry, ACM, and transforms. */
[[nodiscard]] moveit_msgs::srv::GetPlanningScene::Request::SharedPtr
make_full_planning_scene_request();

struct StableCoordinatorState
{
  bool valid{false};
  manipulation_msgs::msg::PlanningSceneCoordinatorState state;
  std::string detail;
};

/** Validate the fail-closed coordinator state/result service contract. */
[[nodiscard]] StableCoordinatorState validate_coordinator_state(
  const manipulation_msgs::msg::PlanningSceneCoordinatorState & state,
  const manipulation_msgs::msg::PlanningSceneOperationResult & result);

/** Pure mapping used by ArmPlanningPort's read-only revision hook. */
[[nodiscard]] SceneRevisionCheck make_planning_revision_check(
  const PlanningSceneRevision & expected,
  const StableCoordinatorState & observed,
  bool stopped);

enum class FullSceneBracketStatus : std::uint8_t
{
  kAccepted,
  kRetry,
  kConflict,
  kInvalid,
};

struct FullSceneBracketCheck
{
  FullSceneBracketStatus status{FullSceneBracketStatus::kInvalid};
  std::string detail;
};

/** Decide whether one state -> full scene -> state bracket is authoritative. */
[[nodiscard]] FullSceneBracketCheck check_full_scene_bracket(
  const StableCoordinatorState & before,
  const moveit_msgs::msg::PlanningScene & scene,
  const StableCoordinatorState & after,
  const std::string & expected_session_id,
  std::uint64_t minimum_revision);

enum class V2SceneMutationKind : std::uint8_t
{
  kStage,
  kAttach,
  kDetach,
};

/** Deterministic, delimiter-safe idempotency key for one exact V2 mutation attempt. */
[[nodiscard]] std::string make_v2_scene_operation_id(
  const std::string & lease_id,
  const std::string & object_id,
  std::uint64_t expected_revision,
  V2SceneMutationKind kind,
  std::uint64_t goal_generation,
  std::uint64_t sequence);

struct SceneDiffRequestBuildResult
{
  bool valid{false};
  manipulation_msgs::srv::ApplyPlanningSceneDiff::Request::SharedPtr request;
  std::string detail;
};

[[nodiscard]] SceneDiffRequestBuildResult make_stage_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  std::uint64_t expected_revision,
  const moveit_msgs::msg::CollisionObject & object);

[[nodiscard]] SceneDiffRequestBuildResult make_attach_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  std::uint64_t expected_revision,
  const std::string & object_id,
  const std::string & attachment_link,
  const std::vector<std::string> & touch_links,
  const moveit_msgs::msg::CollisionObject & link_frame_object);

[[nodiscard]] SceneDiffRequestBuildResult make_detach_scene_request(
  const std::string & lease_id,
  const std::string & operation_id,
  std::uint64_t expected_revision,
  const std::string & object_id,
  const std::string & attachment_link,
  const geometry_msgs::msg::PoseStamped & target_object_pose,
  const moveit_msgs::msg::CollisionObject & attached_geometry);

[[nodiscard]] const moveit_msgs::msg::CollisionObject * find_world_object(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & object_id) noexcept;

[[nodiscard]] const moveit_msgs::msg::AttachedCollisionObject * find_attached_object(
  const moveit_msgs::msg::PlanningScene & scene,
  const std::string & object_id) noexcept;

}  // namespace manipulation::moveit::detail

#endif  // V2_SCENE_ADAPTER_SUPPORT_HPP_
