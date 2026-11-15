# Responsibility: Expose a read-only snapshot of one PlanningScene coordinator session.

# Changes after every coordinator restart. Lease tokens from another session
# are invalid even if their resource names match.
string session_id

# Monotonic within session; increments after each successful scene mutation.
uint64 revision

# Counts belong to this session snapshot and are intended for health and
# capacity diagnostics, not for reconstructing lease ownership.
uint32 active_lease_count
uint32 operation_record_count

# True while one accepted two-phase backend apply is unresolved. The state
# does not claim that MoveIt and the registry are reconciled after a crash.
bool operation_in_progress

# True only when both authoritative MoveIt scene services are discoverable.
# This is an instantaneous readiness observation, not a liveness guarantee.
bool backend_service_ready

# Fail-closed gate for lease validation and scene mutation. It is set after
# startup, backend loss, an ambiguous apply, a readback mismatch or lease
# expiry. A matching authoritative readback clears ordinary scene mismatch, but
# an active resource quarantine requires explicit recovery acknowledgement.
bool reconciliation_required

# Latest machine-readable backend/reconciliation failure. Uses constants from
# PlanningSceneOperationResult; NONE means the latest reconciliation succeeded.
uint16 last_backend_error_code

# Cumulative and currently quarantined lease expiries in this process session.
# The cumulative value is useful for trend diagnostics; only the second value
# describes the current fail-closed recovery condition.
uint64 expired_lease_count
uint64 unreconciled_lease_expiry_count

# Idempotency ledger usage is bounded and records are never silently evicted.
uint32 operation_record_capacity
bool operation_capacity_exhausted

# Resource keys from expired leases remain quarantined until an explicitly
# acknowledged recovery accepts the authoritative scene. Unlike active leases,
# quarantine has no TTL and ordinary readback cannot clear it.
uint32 quarantined_resource_count
string[] quarantined_resource_keys
