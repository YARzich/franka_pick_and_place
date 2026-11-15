# manipulation_msgs

ROS 2 interfaces shared by manipulation workflows and motion-backend adapters.
This package intentionally contains no MoveIt, cuRobo, perception or simulator
implementation code. Production and reference implementations live in
separate backend packages.

The contract is robot-independent, but it is deliberately not planner-neutral:
the coordinated scene services use `moveit_msgs/PlanningScene` as the shared
scene representation. Planner implementations may differ, while every writer
still converges on the same authoritative MoveIt scene contract.

`TransferObjectSequenceV2` is the only public high-level transfer contract.
Version 0.3.0 removed the superseded static-box, perception-specific, inner
transfer and cuRobo-specific actions after repository consumers migrated. An
optional planner backend, including cuRobo, is selected by server deployment
configuration and never changes the action goal or introduces another task
language.

## Contracts

- `TransferObjectStepV2` / `TransferObjectSequenceV2`: an ordered high-level
  intent containing only object identity, target expression, home postcondition,
  and recovery policy. Observation, geometry, grasp, strategy, and backend
  selection are server-owned. The goal has no whole-goal timeout. Feedback pairs
  the typed `ManipulationTaskState.phase` with an append-only waiting reason and
  the active object ID.
- `ServoToPose`: a separate reactive pose-tracking task. It continuously moves
  the server-configured controlled frame toward one target with a mandatory
  non-zero timestamp, resolves cross-frame targets only through exact-stamp TF,
  reports frame-consistent translation and rotation errors, and performs a
  bounded confirmed stop on pause, cancel, stale state, or stale TF. It
  intentionally does not expose a particular robot, controller, or Servo
  implementation.
- `MotionTaskResult`: a motion-only terminal result. Unlike the object-transfer
  result, it carries no object disposition and reports `stop_confirmed` only
  from fresh robot-state velocity samples, never merely from a command or timer.
- `SetTaskPaused`: cooperative pause/resume at safe execution checkpoints.
- PlanningScene lease services: exclusive resource ownership, steady-clock
  TTL, optimistic revision checks and idempotent scene operations for
  multi-node writers.
- `GetPlanningSceneState`: side-effect-free session/revision snapshot used to
  detect coordinator restarts and refresh optimistic concurrency state.
- `ReconcilePlanningScene`: explicit authoritative readback acceptance that
  invalidates all leases/idempotency records and rolls the coordinator session.
  Clearing expired-lease quarantine additionally requires the typed
  `ACCEPT_QUARANTINED_RESOURCES` recovery acknowledgement and the exact key set
  copied from the latest coordinator state.
- `ValidatePlanningSceneLease`: pre-side-effect proof that a lease is active,
  owns every required canonical key and still matches the global revision; its
  remaining TTL is a steady duration, never a ROS timestamp.
- `SystemHealthSnapshot` / `GetSystemHealth`: typed, read-only readiness state
  for automation. Component, level and issue codes are the API; `detail` is
  operator context only.

Action clients must use numeric phase/error/disposition values for automation.
The text fields are operator context and are not a machine-readable API.
Coordinator clients use `PlanningSceneOperationResult.success` as the single
success flag and inspect its typed error code on failure. In particular,
`RESOURCE_QUARANTINED` is distinct from a temporary active-lease conflict.

Pose fields follow REP-103 units and carry explicit `stamp + frame_id`.
PlanningScene lease TTL uses a steady clock so `/clock` resets cannot extend a
lease accidentally.

`PlanningSceneCoordinatorState` is an admission contract, not just a counter
snapshot. `backend_service_ready=false`, `reconciliation_required=true` or
`operation_capacity_exhausted=true` means new mutation-dependent work must not
start. Total lease expiries are historical; `unreconciled_lease_expiry_count`
and the sorted `quarantined_resource_keys` identify the current fail-closed
condition. Quarantine has no TTL and a matching readback cannot clear it.
Recovery through `ReconcilePlanningScene` always rolls `session_id`,
invalidating stale leases.

Health snapshots use the ROS-clock `sample_stamp` only for correlation because
it may pause or reset. Their `sequence` is monotonic within a monitor process,
and every component observation age is measured with a steady clock. The
readiness policy is deliberately conservative: `motion_ready` is true only
when every required component is `LEVEL_OK`; startup and any `WARN` block a new
motion command.

Perception health uses backend-neutral component IDs for stereo, detection and
pose estimation plus generic unavailable/delayed/stale/reported-level issue
codes. Diagnostic task names stay in monitoring configuration rather than the
wire message, so this interface package remains independent of perception
implementations.

## Wire Contract Snapshot

The authored files under `msg/`, `srv/`, and `action/` are the canonical current
contract. `docs/public_wire_contract.v2.json` is its exact reviewed semantic
snapshot. `docs/public_wire_contract.v1.json` is an immutable archive from
before the breaking migration; its byte-level SHA-256 is checked, but removed
types are deliberately not generated from that file. Snapshot entries record
ordered fields, structural types, explicit defaults, and ordered constants with
their types and values. Source comments and formatting are intentionally
excluded because they do not change generated ROS types; behavior documented by
those comments remains governed by this README and the repository
[change-control rules](../docs/CHANGE_CONTROL.md).

The contract test parses sources with the official `rosidl_adapter` parser and
compares the complete source inventory with V2. It also checks every generated
Python class through `SLOT_TYPES`, proves retained interfaces match V1, and
prevents removed historical types from re-entering the current snapshot.

The snapshot helper checks by default and refuses to write unless the reviewer
provides an explicit acknowledgement. Run it from the package root only after
reviewing the public change:

```bash
python3 test/update_public_wire_contract.py --write \
  --acknowledge-reviewed-public-change
```

Updating the snapshot records a reviewed public-surface change; it does not make
a breaking edit compatible. Additive constants must be appended without
renumbering or reusing a value, and consumers must tolerate unknown future
values.

When an interface is added, register it in `CMakeLists.txt` and protect its exact
source/generated shape with reviewed contract evidence. Keep the previous
interface available until the migration gates in
[`docs/CHANGE_CONTROL.md`](../docs/CHANGE_CONTROL.md)
are satisfied.

## Migration

The completed V1-to-V2 endpoint mapping, repository consumer inventory and
breaking-removal record are in [`MIGRATION.md`](MIGRATION.md) and
[`docs/transfer_v1_removal_inventory.md`](docs/transfer_v1_removal_inventory.md).
Clients must migrate to the V2 endpoint; no removed endpoint changes wire type
in place and no compatibility bridge is installed.
