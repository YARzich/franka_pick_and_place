// Responsibility: Defines exclusive resource leases and idempotent operation replay for scene
//   owners.

#ifndef MANIPULATION__SCENE__SCENE_LEASE_REGISTRY_HPP_
#define MANIPULATION__SCENE__SCENE_LEASE_REGISTRY_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace manipulation::scene
{

/// Stable, machine-readable outcomes returned by the lease registry.
enum class RegistryError : std::uint16_t
{
  kNone = 0,
  kInvalidRequest = 100,
  kResourceConflict = 101,
  kLeaseNotFound = 102,
  kLeaseExpired = 103,
  kNotOwner = 104,
  kRevisionMismatch = 105,
  kBackendFailed = 106,
  kTimeout = 107,
  kSessionMismatch = 108,
  kIdempotencyConflict = 109,
  kOperationInProgress = 110,
  kOperationTicketInvalid = 111,
  kOperationCapacityExceeded = 112,
  kReconciliationRequired = 113,
  kDependencyUnavailable = 114,
  kResourceQuarantined = 115,
  kInternalError = 255,
};

/// Common status returned by every state-changing registry operation.
struct RegistryStatus
{
  bool success{false};
  RegistryError error{RegistryError::kInternalError};
  std::uint64_t revision{0};

  // Operator context only. Automation must branch on `error`.
  std::string message;
};

/// Input for acquiring exclusive ownership of one or more resource keys.
struct AcquireLeaseRequest
{
  std::string owner_id;
  std::vector<std::string> resource_keys;
  std::chrono::steady_clock::duration requested_ttl{};
};

/// Lease acquisition status and the opaque token needed by later calls.
struct AcquireLeaseResult
{
  RegistryStatus status;
  std::string lease_token;
  std::chrono::steady_clock::duration granted_ttl{};

  // Canonical existing/quarantined key that overlaps the request. It may
  // differ textually from the requested key under object-lifecycle semantics.
  std::string conflicting_resource_key;
};

/// Result of renewing an active lease from the registry's current time.
struct RenewLeaseResult
{
  RegistryStatus status;
  std::chrono::steady_clock::duration granted_ttl{};
};

/// Read-only pre-side-effect check for one active lease and resource subset.
struct ValidateLeaseRequest
{
  std::string lease_token;
  std::vector<std::string> required_resource_keys;
  std::uint64_t expected_revision{0};
};

/// Validation status plus the lease's remaining steady-clock lifetime.
struct ValidateLeaseResult
{
  RegistryStatus status;
  std::string session_id;
  std::chrono::steady_clock::duration remaining_ttl{};
};

/// Input for authorizing one idempotent backend PlanningScene mutation.
struct BeginOperationRequest
{
  std::string lease_token;
  std::string operation_id;
  std::uint64_t expected_revision{0};
  std::vector<std::string> resource_keys;

  // Deterministic digest of the complete backend request, including payload.
  std::string request_digest;
};

/// Tells a caller whether it should execute the backend or reuse a result.
enum class OperationDisposition
{
  kRejected,
  kReady,
  kReplay,
};

/// Explicit recovery decision applied atomically with a session rebase.
enum class QuarantineResolution
{
  kPreserve,
  kAccept,
};

/// Authorization result for a two-phase PlanningScene operation.
struct BeginOperationResult
{
  RegistryStatus status;
  OperationDisposition disposition{OperationDisposition::kRejected};
  std::string operation_ticket;
};

/// Outcome produced by the external PlanningScene backend.
struct BackendOperationResult
{
  bool success{false};
  std::string message;

  // Used only for a failed backend outcome. Existing callers that initialize
  // `{success, message}` keep the original `kBackendFailed` behavior.
  RegistryError failure_error{RegistryError::kBackendFailed};
};

/// Snapshot intended for diagnostics; all values are copied atomically.
struct RegistrySnapshot
{
  std::string session_id;
  std::uint64_t revision{0};
  std::size_t active_lease_count{0};
  std::size_t operation_record_count{0};
  std::size_t operation_record_capacity{0};
  bool operation_capacity_exhausted{false};
  bool operation_in_progress{false};

  // Cumulative session counter, including expiries detected by other calls.
  std::uint64_t expired_lease_count{0};

  // Canonical keys retained after lease expiry. They are sorted for stable
  // diagnostics and remain blocked until an explicit session rebase.
  std::vector<std::string> quarantined_resource_keys;
};

/**
 * Owns exclusive resource leases and idempotency records for one scene session.
 *
 * Ownership and lifetime:
 * - The registry owns all lease and operation metadata. Inputs and outputs are
 *   copied; callers retain ownership of backend PlanningScene data.
 * - The registry never calls MoveIt or any other backend. A caller executes the
 *   backend between `begin_operation()` and `complete_operation()`.
 * - An in-flight operation pins its lease even after TTL expiry. The caller
 *   must always complete the ticket so ownership can be released safely.
 *
 * Thread safety:
 * - Every public member function is thread-safe.
 * - At most one backend operation is authorized at a time, preserving the
 *   global expected-revision invariant without holding a mutex across backend
 *   work.
 * - The injected clock callback is invoked while the registry mutex is held;
 *   it must be fast, monotonic and must not call back into this object.
 *
 * Revision invariant:
 * - The revision is monotonically increasing and advances exactly once for
 *   each successfully completed backend mutation. Lease changes and failed
 *   backend operations do not advance it.
 */
class SceneLeaseRegistry final
{
public:
  using Clock = std::chrono::steady_clock;
  using Duration = Clock::duration;
  using TimePoint = Clock::time_point;
  using NowFunction = std::function<TimePoint()>;

  /// Construction settings fixed for the lifetime of one coordinator session.
  struct Options
  {
    // Must be unique for every coordinator process lifetime, normally a UUID.
    std::string session_id;
    std::uint64_t initial_revision{0};

    // Completed records are retained for the entire session. New IDs are
    // rejected at this bound so idempotency is never silently weakened.
    std::size_t max_operation_records{4096};
  };

  /**
   * Construct an empty registry for `options.session_id`.
   *
   * The session ID must be unique for this coordinator lifetime, non-empty and
   * contain no whitespace or control characters. `max_operation_records` must
   * be positive. Invalid options or an empty clock function throw
   * `std::invalid_argument`.
   */
  explicit SceneLeaseRegistry(
    Options options,
    NowFunction now = [] () {return Clock::now();});

  ~SceneLeaseRegistry();

  SceneLeaseRegistry(const SceneLeaseRegistry &) = delete;
  SceneLeaseRegistry & operator=(const SceneLeaseRegistry &) = delete;
  SceneLeaseRegistry(SceneLeaseRegistry &&) = delete;
  SceneLeaseRegistry & operator=(SceneLeaseRegistry &&) = delete;

  /**
   * Acquire all requested canonical resources atomically and exclusively.
   *
   * World and attached keys for the same object form one lifecycle ownership
   * scope and conflict with ACM keys containing that object across leases.
   * Related keys in this request remain co-ownable because acquisition is
   * atomic. This prevents one owner from implicitly erasing another owner's
   * ACM entries when an object is removed or moved between world and robot.
   *
   * A semantically overlapping key quarantined after lease expiry returns
   * `kResourceQuarantined`, even though no active lease owns it. Quarantine
   * has no TTL and is cleared only by `rebase_session()` after explicit
   * recovery acknowledgement.
   */
  AcquireLeaseResult acquire_lease(const AcquireLeaseRequest & request);

  /**
   * Renew an active lease to expire `requested_ttl` after the current time.
   *
   * Renewal never revives a lease whose TTL has already elapsed.
   */
  RenewLeaseResult renew_lease(
    const std::string & lease_token,
    Duration requested_ttl);

  /**
   * Validate lease lifetime, resource ownership and global scene revision.
   *
   * This call never authorizes or applies a PlanningScene mutation. Expired
   * leases may be purged as registry housekeeping. A successful remaining TTL
   * is measured by the registry's steady clock and is safe to compare with a
   * caller's remaining steady duration.
   */
  ValidateLeaseResult validate_lease(const ValidateLeaseRequest & request);

  /**
   * Release every resource owned by a lease.
   *
   * Release is rejected while that lease has an in-flight backend operation.
   */
  RegistryStatus release_lease(const std::string & lease_token);

  /**
   * Validate and reserve one idempotent PlanningScene operation.
   *
   * `request_digest` must deterministically cover the complete backend request;
   * the registry cannot inspect backend-specific payloads. Repeating the same
   * operation ID and request returns `kReplay`. Reusing the ID for different
   * content returns `kIdempotencyConflict`.
   */
  BeginOperationResult begin_operation(const BeginOperationRequest & request);

  /**
   * Record an external backend outcome and release the in-flight reservation.
   *
   * A successful outcome increments the revision once. A failed outcome is
   * retained and replayed with `BackendOperationResult::failure_error`, which
   * may be `kBackendFailed`, `kTimeout` or `kInternalError`. The ticket is
   * opaque and can be completed only once.
   */
  RegistryStatus complete_operation(
    const std::string & operation_ticket,
    const BackendOperationResult & backend_result);

  /**
   * Invalidate leases, quarantines and idempotency records after reconciliation.
   *
   * The reset is rejected while an operation is active. Expiry is evaluated
   * under the same registry lock as the reset; quarantined resources are
   * cleared only with `QuarantineResolution::kAccept` and an exact expected-key
   * semantic set; input order is ignored. `new_session_id` must satisfy the
   * same token rules as construction and must differ from the old session.
   * Revision remains monotonic across the in-process epoch rollover.
   */
  RegistryStatus rebase_session(
    const std::string & new_session_id,
    QuarantineResolution quarantine_resolution = QuarantineResolution::kPreserve,
    const std::vector<std::string> & expected_quarantined_resource_keys = {});

  /**
   * Quarantine resources whose steady-clock lease TTL elapsed and return tokens.
   *
   * A lease pinned by an in-flight operation is intentionally not removed.
   * Every expired lease key remains unavailable beyond TTL until an explicit
   * session rebase. This fail-closed transition does not mutate PlanningScene.
   */
  std::vector<std::string> expire_leases();

  /**
   * Return a consistent, copied diagnostic snapshot.
   *
   * This read-only call does not expire leases. Coordinator adapters should
   * call `expire_leases()` first so they can diagnose every ownership expiry.
   */
  RegistrySnapshot snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__SCENE_LEASE_REGISTRY_HPP_
