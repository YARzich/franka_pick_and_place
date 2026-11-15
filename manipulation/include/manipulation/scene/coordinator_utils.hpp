// Responsibility: Exposes coordinator contract validation, identifiers, digests and time
//   conversions.

#ifndef MANIPULATION__SCENE__COORDINATOR_UTILS_HPP_
#define MANIPULATION__SCENE__COORDINATOR_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <manipulation_msgs/msg/planning_scene_coordinator_state.hpp>
#include <manipulation_msgs/msg/planning_scene_operation_result.hpp>
#include <manipulation_msgs/srv/apply_planning_scene_diff.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "manipulation/scene/scene_lease_registry.hpp"

namespace manipulation::scene
{

/// Result of validating and converting a ROS duration into a steady TTL.
struct SteadyDurationResult
{
  bool valid{false};
  std::chrono::steady_clock::duration value{};
  std::string message;
};

/**
 * Convert a ROS duration into a positive steady-clock duration.
 *
 * Negative seconds, nanoseconds outside `[0, 1e9)` and a zero total duration
 * are rejected. The conversion is independent of ROS or simulation time.
 */
SteadyDurationResult parse_positive_steady_duration(
  const builtin_interfaces::msg::Duration & duration);

/// Convert a positive steady duration to a ROS message when it fits `int32 sec`.
std::optional<builtin_interfaces::msg::Duration> to_duration_message(
  std::chrono::steady_clock::duration duration);

/// Backend states understood by the coordinator's bounded MoveIt call.
enum class BackendCallOutcome
{
  kSuccess,
  kUnavailable,
  kTimeout,
  kRejected,
  kInternalError,
};

/// Convert a backend call state into the registry's stable completion contract.
BackendOperationResult make_backend_operation_result(
  BackendCallOutcome outcome,
  std::string message);

/**
 * Map every registry error explicitly to `PlanningSceneOperationResult`.
 *
 * The explicit switch prevents accidental wire-value drift when either enum
 * grows. Unknown values fail closed as `INTERNAL_ERROR`.
 */
std::uint16_t to_interface_error(RegistryError error);

/// Convert a registry status into the custom ROS service result message.
manipulation_msgs::msg::PlanningSceneOperationResult to_interface_result(
  const RegistryStatus & status);

/// Convert a registry snapshot when its counters fit the wire-level uint32 fields.
std::optional<manipulation_msgs::msg::PlanningSceneCoordinatorState>
to_interface_state(const RegistrySnapshot & snapshot);

/// Validation result for the coordinator's deliberately narrow diff subset.
struct SceneDiffValidationResult
{
  bool valid{false};
  std::vector<std::string> canonical_resource_keys;
  std::string message;
};

/**
 * Validate that every supported scene mutation has exact lease-key coverage.
 *
 * Supported mutations are deterministic world/attached ADD and REMOVE plus a
 * square symmetric ACM submatrix. ADD always carries explicit geometry.
 * Attached ADD uses its link frame and cannot share an ID with a world
 * mutation; attached REMOVE is paired with an explicit world ADD/REMOVE.
 * Attached transitions require both `attached:<id>` and `world:<id>` leases.
 * Non-REMOVE objects require an explicit REP-compliant `header.frame_id`
 * without a leading slash. ACM diagonal cells are structural placeholders and
 * do not create self-pair resources.
 * Global robot state, octomap, colors, transforms, padding, scale and ACM
 * defaults are rejected because they cannot be assigned safely to the current
 * resource-key model.
 */
SceneDiffValidationResult validate_scene_diff_resources(
  const moveit_msgs::msg::PlanningScene & scene_diff,
  const std::vector<std::string> & declared_resource_keys);

/**
 * Return a lowercase SHA-256 digest of the complete serialized apply request.
 *
 * Resource keys are sorted in a copy because they form a semantic set. Every
 * other field, including the full PlanningScene payload, is encoded through
 * generated ROSIDL YAML in schema order. This avoids unspecified CDR alignment
 * padding while preserving every field value.
 *
 * OpenSSL or ROS serialization failures throw `std::runtime_error`.
 */
std::string digest_apply_request(
  const manipulation_msgs::srv::ApplyPlanningSceneDiff::Request & request);

/**
 * Hash the authoritative coordinator-owned PlanningScene domains.
 *
 * World objects, attached objects and the allowed-collision matrix are sorted,
 * stripped of transport timestamps and pose values are canonicalized at
 * picometre-scale resolution before serialization. Robot joints,
 * octomap, transforms and other non-owned domains are deliberately excluded.
 * The result is stable across harmless MoveIt response ordering changes and is
 * used only for readback reconciliation, never as a resource identifier.
 */
std::string digest_authoritative_scene(
  const moveit_msgs::msg::PlanningScene & scene);

/**
 * Return the canonical coordinator-owned part of an authoritative scene.
 *
 * The returned message contains only world objects, attached objects and the
 * ACM. Transport stamps and ordering differences are removed. Malformed
 * matrices, duplicate IDs or invalid geometry poses throw `std::runtime_error`.
 */
moveit_msgs::msg::PlanningScene canonical_authoritative_scene(
  const moveit_msgs::msg::PlanningScene & scene);

/// Deterministic result of applying the coordinator's supported diff subset.
struct SceneEvolutionResult
{
  bool valid{false};
  moveit_msgs::msg::PlanningScene expected_scene;
  moveit_msgs::msg::PlanningScene backend_diff;
  std::string message;
};

/**
 * Evolve a complete owned-scene snapshot using MoveIt-compatible semantics.
 *
 * The helper accepts the deliberately deterministic subset enforced by the
 * coordinator: explicit world/attached ADD, world REMOVE, and attached REMOVE
 * paired with a world ADD/REMOVE for the same ID. APPEND, MOVE, implicit
 * geometry transfer and cross-frame transforms are rejected. A pair-scoped
 * ACM request is merged into the complete baseline matrix; `backend_diff`
 * therefore carries a full ACM because MoveIt replaces, rather than merges,
 * a non-empty matrix in `PlanningScene::setPlanningSceneDiffMsg()`.
 */
SceneEvolutionResult evolve_authoritative_scene(
  const moveit_msgs::msg::PlanningScene & authoritative_scene,
  const moveit_msgs::msg::PlanningScene & requested_diff,
  const std::string & planning_frame);

/// Generate a random RFC 4122 version-4 session UUID using OpenSSL RAND_bytes.
std::string generate_session_id();

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__COORDINATOR_UTILS_HPP_
