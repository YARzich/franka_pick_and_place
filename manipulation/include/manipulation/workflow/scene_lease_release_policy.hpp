// Responsibility: Decides whether scene leases may be released after scene or task outcomes.

#ifndef MANIPULATION__WORKFLOW__SCENE_LEASE_RELEASE_POLICY_HPP_
#define MANIPULATION__WORKFLOW__SCENE_LEASE_RELEASE_POLICY_HPP_

#include <cstdint>

#include "manipulation/task/task_status.hpp"

namespace manipulation::workflow
{

/** Result of evaluating whether a terminal disposition permits lease release. */
enum class SceneLeaseReleaseDecision
{
  kRelease,
  kRetain,
};

/**
 * Classify a wire-level terminal object disposition conservatively.
 *
 * A lease is releasable only when the backend confirms that it is idle or the
 * object is detached at a known source/target pose. Holding, uncertain,
 * unknown, and future disposition values retain ownership until TTL expiry or
 * an explicit recovery owner resolves the scene.
 */
[[nodiscard]] SceneLeaseReleaseDecision scene_lease_release_decision(
  std::uint8_t final_disposition) noexcept;

/**
 * Classify a domain-level terminal object disposition conservatively.
 *
 * This overload is the policy boundary for pure workflows that have not
 * serialized their terminal evidence to a ROS message.
 */
[[nodiscard]] SceneLeaseReleaseDecision scene_lease_release_decision(
  manipulation::task::ObjectDisposition final_disposition) noexcept;

/**
 * Classify complete terminal evidence, including physical-command ownership.
 *
 * Even a normally releasable object disposition retains the scene lease when
 * an accepted physical command has not reached a proven terminal boundary.
 */
[[nodiscard]] SceneLeaseReleaseDecision scene_lease_release_decision(
  const manipulation::task::TaskOutcome & outcome) noexcept;

/**
 * Classify an explicitly failed source-object ADD response.
 *
 * Release is allowed only for coordinator errors that prove rejection before
 * backend dispatch. Timeout, in-progress, backend, reconciliation, internal,
 * and future error values retain ownership because a late mutation cannot be
 * excluded from the wire result alone.
 */
[[nodiscard]] SceneLeaseReleaseDecision scene_add_failure_release_decision(
  std::uint16_t error_code) noexcept;

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__SCENE_LEASE_RELEASE_POLICY_HPP_
