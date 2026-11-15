<!-- Responsibility: Record the public ROS and CMake contracts of manipulation. -->
# manipulation API

## CMake

```cmake
find_package(manipulation REQUIRED)

target_link_libraries(my_backend
  manipulation::task
  manipulation::servo
  manipulation::skill
  manipulation::world
  manipulation::recording
  manipulation::workflow
)
```

Installed headers use `manipulation/<layer>/...`; C++ symbols use
`manipulation::<layer>`.

`manipulation::transport` provides the installed
`manipulation/transport/deadline_service_call.hpp` helper for one stop-aware
typed service request under an absolute steady deadline. The caller must place
the client's callback group in a concurrently spinning executor and pass that
node's `rclcpp::Context`; timeout, cancel, and context-shutdown paths remove the
matching pending request before returning. Context shutdown is reported
separately as `ServiceCallStatus::kShutdown`.

## Skill, World State, and Recording

All types in this section are ROS-backend neutral and have no hidden global
state.

| CMake target | Main public headers | Contract |
| --- | --- | --- |
| `manipulation::servo` | `servo/servo_to_pose_types.hpp`, `servo/servo_to_pose_state_machine.hpp` | Pure deterministic fixed-pose tracking and safe-stop policy. |
| `manipulation::skill` | `skill/skill_descriptor.hpp`, `skill/resource_registry.hpp` | Immutable skill metadata, startup capability validation, and atomic process-local resource ownership. |
| `manipulation::world` | `world/world_state_snapshot.hpp` | Validated immutable stamped rigid-object snapshot with provenance and an opaque token. |
| `manipulation::recording` | `recording/episode_event_sink.hpp` | Validated lifecycle events and a nonblocking, best-effort enqueue boundary. |
| `manipulation::workflow` | `workflow/skill_runtime.hpp` | Composition of one descriptor with available capabilities, a shared resource registry, and an optional event sink. |

Logical identifiers are lowercase slash-separated values validated by
`skill/logical_id.hpp`. A segment begins with a lowercase ASCII letter and may
continue with lowercase letters, digits, `_`, `.`, or `-`.

`PresetIdentity` contains a readable `preset_id`, positive schema version, and
lowercase SHA-256 digest of canonical preset content. The digest identifies the
effective configuration; changing a value without renaming the preset still
produces a different identity.

`CapabilitySet` is sorted, unique, and immutable after construction.
`SkillRuntime::create()` rejects a descriptor when the deployment does not
provide every required capability. The standard vocabulary currently includes
planned pose execution, Cartesian paths, gripper-width commands, coordinated
scene mutation, exact-stamp TF, rigid-object world state, and reactive
`motion/servo_pose` execution. The generic package defines the capability name
and pure policy; a backend package supplies the ROS/controller adapter.

`ResourceRegistry::try_acquire()` is atomic and nonblocking. It either returns
one move-only RAII lease covering every claim or returns a typed error and no
partial ownership. The registry provides no waiting, preemption, TTL,
controller switching, or cross-process arbitration. A read-only skill may have
no claims and succeeds without a lease.

`WorldStateSnapshot::create()` validates non-zero stamped poses, provenance,
object IDs, optional normalized confidence, and duplicate identity. Objects are
stored in deterministic ID order. An empty object collection is valid.

`EpisodeEventSink::try_enqueue()` must return immediately and perform no disk
or network I/O. Events identify the episode, skill, exact preset, steady elapsed
time, task state, optional terminal outcome, and optional world-state token.
`SkillRuntime::try_record()` contains validation, allocation, and sink errors;
recording therefore cannot fail an action. `NullEpisodeEventSink` is the
default when no recorder is injected.

## ServoToPose State Machine

Headers:

- `manipulation/servo/servo_to_pose_types.hpp`;
- `manipulation/servo/servo_to_pose_state_machine.hpp`.

CMake target: `manipulation::servo`

Namespace: `manipulation::servo`

`ServoToPoseStateMachine` is a pure synchronous state machine. It has no ROS or
MoveIt dependency and never performs side effects itself. The owner calls
`step()` with one explicit `ServoToPoseStepInput` and executes the returned
`ServoToPoseCommand`:

| Command | Owner responsibility |
| --- | --- |
| `kPublishTarget` | Publish the already prepared fixed target once. |
| `kRequestResume` | Ask the reactive backend to enter its running state. |
| `kRequestPause` | Ask the backend to enter its paused state. |
| `kNone` | Perform no backend command during this step. |

The configuration defines position and orientation tolerances, the required
number of fresh consecutive target samples, stopped-velocity tolerance, the
required number of fresh stopped samples, and a bounded halt timeout. Pose and
velocity samples carry monotonically changing sequence numbers; polling the
same cached observation repeatedly does not create new evidence.

The machine starts with the backend expected to be paused. A normal session
resumes, publishes one fixed target while tracking, and enters the halt path
after confirmed arrival. Pause, cancel, deadline, stale feedback, invalid
feedback, safety failure, and backend failure use the same halt path. The
machine stops requesting target publication, requires fresh low-velocity
evidence, and requires a confirmed paused backend before it reports a safe
pause or terminal result. If the backend was already paused when setup failed,
fresh low-velocity evidence can still prove the terminal stop without a
redundant pause request. Normal success then requires new distinct pose samples
captured after the confirmed pause; stale, invalid, or out-of-tolerance final
pose evidence produces a typed failure while preserving the confirmed stop.

`snapshot()` is side-effect free. The class is intentionally non-copyable and
non-movable so one owner controls one execution history. ROS action lifecycle,
TF, message transport, controller calls, resource admission, and worker-thread
ownership belong to a backend integration such as `manipulation_moveit`.

## PlanningScene Coordinator

Executable: `planning_scene_coordinator`

Default node name: `/planning_scene_coordinator`

| Relative service | Type |
| --- | --- |
| `~/acquire_lease` | `manipulation_msgs/srv/AcquirePlanningSceneLease` |
| `~/renew_lease` | `manipulation_msgs/srv/RenewPlanningSceneLease` |
| `~/release_lease` | `manipulation_msgs/srv/ReleasePlanningSceneLease` |
| `~/validate_lease` | `manipulation_msgs/srv/ValidatePlanningSceneLease` |
| `~/apply_scene_diff` | `manipulation_msgs/srv/ApplyPlanningSceneDiff` |
| `~/get_state` | `manipulation_msgs/srv/GetPlanningSceneState` |
| `~/reconcile` | `manipulation_msgs/srv/ReconcilePlanningScene` |

| Parameter | Type | Default | Semantics |
| --- | --- | --- | --- |
| `use_sim_time` | `bool` | `false` | Selects the ROS clock used by TF and stamped diagnostics; lease and backend deadlines remain steady-clock based. |
| `backend_service` | `string` | `/apply_planning_scene` | Non-empty authoritative `moveit_msgs/srv/ApplyPlanningScene` writer endpoint. |
| `readback_service` | `string` | `/get_planning_scene` | Non-empty authoritative `moveit_msgs/srv/GetPlanningScene` readback endpoint used to prove mutations and reconcile ambiguity. |
| `planning_frame` | `string` | `world` | Canonical relative frame required on public world-object mutations. |
| `backend_planning_frame` | `string` | `world` | Canonical relative MoveIt model frame; the coordinator explicitly transforms public objects into this frame. |
| `robot_description_topic` | `string` | `/robot_description` | Transient-local URDF source used for fixed-root model-frame verification. |
| `verify_fixed_root_model_frame` | `bool` | `true` | When true, backend readiness remains false until the URDF fixed root equals `backend_planning_frame`; disabling it is an explicit profile decision. |
| `backend_service_timeout_sec` | `double` | `5.0` | Positive finite steady deadline covering pre-dispatch work and one backend apply response. A timed-out dispatched mutation becomes pending reconciliation, not an assumed failure. |
| `readback_service_timeout_sec` | `double` | `5.0` | Positive finite steady deadline covering discovery and response for one authoritative scene readback. |
| `lease_expiry_check_period_sec` | `double` | `0.5` | Positive finite wall/steady period for lease-expiry maintenance. |
| `reconciliation_check_period_sec` | `double` | `1.0` | Positive finite wall/steady period for settling pending operations and checking scene consistency. |
| `session_id` | `string` | empty | Coordinator-session identity; empty generates a fresh UUID for this process lifetime. A configured value must satisfy the registry identifier contract. |
| `initial_revision` | `int64` | `0` | Non-negative initial scene revision, normally zero for a fresh session. |
| `max_operation_records` | `int64` | `4096` | Positive bound no greater than `uint32` max. Completed idempotency records stay for the full session; new operation IDs are rejected at capacity. |
| `diagnostic_updater.period` | `double` | `1.0` | Standard `diagnostic_updater` publication period in seconds for coordinator status. |
| `diagnostic_updater.use_fqn` | `bool` | `false` | Prefix standard diagnostic status names with the fully qualified node name instead of the node basename. |

Lease TTL, backend timeouts, reconciliation, and operation-ledger maintenance
use a steady clock. ROS time is used only for stamped transport data and
diagnostics.

Lease exclusion follows PlanningScene side effects, not only exact key text.
`world:<id>` and `attached:<id>` are one object-lifecycle ownership scope and
conflict across leases with every `acm:` pair containing `<id>`. One atomic
lease may and normally should acquire all related keys together. This prevents
object removal or attachment transitions from implicitly erasing ACM state
owned by another workflow. Unrelated objects and distinct ACM pairs remain
independently leasable. The same overlap rule applies to expired-resource
quarantine until explicit recovery acknowledgement.

`planning_frame` and `backend_planning_frame` form the scene-frame profile. The
installed neutral profile sets both to `world`; a robot application overrides
them when its public base frame differs.

## V2 Transfer Workflow

CMake target: `manipulation::workflow`

Public facade header:
`manipulation/workflow/transfer_object_sequence_v2_action_server.hpp`

Pure orchestration header:
`manipulation/workflow/resolved_transfer_workflow.hpp`

`TransferObjectSequenceV2ActionServer` is the only transfer action facade
implemented by this package. It owns single-goal admission, cooperative pause,
exact-goal cancellation, a managed worker, typed feedback, and exactly-once
terminal delivery. Its backend supplies three explicit callbacks:

- side-effect-free admission for an immutable V2 goal;
- execution on the managed worker with one borrowed
  `TransferSequenceTaskContext`;
- non-blocking stop propagation.

The facade has no task-configurable whole-goal timeout. Its heartbeat is an
implementation detail, while cancel and shutdown interrupt cooperative backend
waits through `TaskStopCause`.

`ResolvedTransferWorkflow` maps each intent-only step through an injected
`ObjectCatalog`, `ObservationProvider`, `GraspProvider`, coordinated scene port,
and selected `TransferExecutionStrategy`. It owns ordered orchestration and
goal-scoped cleanup, but no ROS node, planner, controller, gripper, TF buffer,
detector, or robot model. `make_resolved_transfer_workflow_callbacks()` is the
adapter between this pure workflow and the V2 action lifecycle.

`ObjectProfile::orientation_symmetries()` optionally declares unique unit
quaternions for proper rotations in the object's local frame that leave its
physical state equivalent. A non-empty collection contains the identity exactly
once. `TARGET_INITIAL_SOURCE` keeps the recorded source position and selects the
orientation `initial_source * local_symmetry` nearest to the current raw
observation. This changes only the resolved return target: the observation used
for staging and grasp generation and every `TARGET_ABSOLUTE_POSE` payload remain
unchanged. An empty symmetry collection preserves the recorded source
orientation exactly.

Goal-scoped scene cleanup is conditional. `ResolvedTransferWorkflow` retains
the lease by default and enables release only after explicit terminal
finalization. After strategy dispatch, the shared
`scene_lease_release_policy` requires both a confirmed safe-idle/source/target
object disposition and `ResourceReleaseDisposition::kReleaseSafe`.
Holding, unknown, scene-uncertain, ambiguous post-dispatch scene mutation, and
physical-command quarantine evidence keep the lease active for coordinator TTL
or recovery. A pre-strategy cancel/failure may release when the workflow knows
that no ambiguous scene mutation occurred.
Lease release removes ownership, not physical world geometry. An observed object
remains in the authoritative world after cancellation before grasp; successful
detach leaves it at the confirmed target. Removing either object requires an
explicit coordinated scene mutation, not implicit lease cleanup.

Current-step progress remains monotonic until a new sequence step becomes active.
Workflow waits and transport-only waits without checkpoint metadata preserve the
last checkpoint/retry/recovery counters, including during final scene readback.
Explicit strategy feedback remains authoritative for those counters; starting a
new sequence step clears the previous step's feedback.
Target geometry resolution is a private pure policy; lifecycle and lease
ownership remain in the workflow.

Sequence progress uses `TransferSequenceFeedbackSnapshot`; terminal evidence
uses `TransferSequenceExecution`, containing the aggregate result, successful
prefix, optional failed step, each started step outcome, and matching steady
elapsed durations. Invalid vector sizes, indices, enum values, or contradictory
success evidence fail closed as `INTERNAL_ERROR` before ROS delivery while
preserving any conservative physical-command quarantine requirement.
An exception in a started step preserves the completed prefix and identifies
the failing step. A strategy outcome already returned remains the source of
physical disposition and recovery evidence. If the strategy throws before
returning that evidence, the workflow requests stop and requires command-resource
quarantine. Scene ownership is retained for every exceptional step exit.

The lifecycle implementation is independent of the generated action type.
`detail/transfer_sequence_session.hpp` linearizes pause, stop, short backend
operations, and completion under one mutex. `TransferSequenceGoalReservation`
is a process-local nonblocking RAII gate retained until terminal delivery is
reliable and the worker is reusable. It provides no queuing, preemption, TTL,
or cross-process arbitration.

Downstream ports receive `TransferExecutionContext`. `run_operation()` is the
only admission point for a short planning, physical-command, or scene-mutation
dispatch: it is linearized with pause, cancel, shutdown, and lifecycle
completion. `heartbeat()` observes stop and services goal-scoped renewal and
throttled diagnostics without applying pause; this is the correct seam while a
physical command is already accepted. The narrower
`run_maintenance_operation()` ignores pause but remains atomic with
cancel/shutdown/deadline and is reserved for lease renewal. Cooperative
`checkpoint()` and completed `wait_until()` boundaries apply pending pause
before pure workflow execution advances. At that safe boundary, feedback
acknowledges `paused=true` while lease renewal continues. Resume and cancel clear
the acknowledgement; an accepted command still reports only `pause_pending`
until it reaches the next safe boundary.

Non-physical waits emit typed `enter`, throttled `waiting`, and `exit` records.
An exit carries `ready`, `canceled`, `shutdown`, or `error` plus the last
meaningful diagnostic; lifecycle transitions never replace that evidence with
a generic “reason changed” message.

The V2 distributed contracts come from
`manipulation_msgs/action/TransferObjectSequenceV2`,
`manipulation_msgs/msg/TransferObjectStepV2`, and `SetTaskPaused`. Robot and
scenario packages own concrete endpoint names and composition profiles.

## Frame ID Validation

Header: `manipulation/validation/frame_id.hpp`

`is_canonical_relative_frame_id()` is the shared robot-neutral syntax boundary
for TF, MoveIt, perception, and workflow configuration. It accepts nested
relative names such as `robot/camera_optical_frame` and rejects empty names,
leading/trailing separators, repeated separators, whitespace, and control
characters. This is an explicit fail-closed project contract; REP-105 defines
frame semantics but does not prescribe this complete grammar.

The older `is_relative_rep105_frame_id()` name remains as a source-compatible
alias. New code should use the precise canonical-policy name.

## CollisionObject Structural Validation

Header: `manipulation/scene/collision_object_validation.hpp`

CMake target: `manipulation::scene_support`

`validate_explicit_collision_object()` is the robot-neutral trust-boundary
check for complete world-object `ADD` messages. It validates identity, frame,
normalized timestamp, explicit non-degenerate geometry, pose cardinality,
rigid poses, and unique subframes without ROS calls or mutation. A zero stamp
is accepted because authoritative MoveIt readback may use it as a normalized
sentinel; `validate_staged_collision_object()` adds the perception-specific
requirements for an exact planning frame and a non-zero observation stamp.

## Health Monitor

Executable: `system_health_monitor`

Default node name: `/system_health_monitor`

| Interface | Type |
| --- | --- |
| `~/health` | `manipulation_msgs/msg/SystemHealthSnapshot` |
| `~/get_health` | `manipulation_msgs/srv/GetSystemHealth` |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

Core publication and dependency bindings:

| Parameter | Type | Default | Semantics |
| --- | --- | --- | --- |
| `use_sim_time` | `bool` | `false` | Selects the ROS clock used for sample stamps, joint-stamp comparison and TF; freshness and request deadlines use a steady clock. |
| `diagnostic_updater.period` | `double` | `1.0` | Standard diagnostic publication period in seconds. |
| `diagnostic_updater.use_fqn` | `bool` | `false` | Prefix standard diagnostic status names with the fully qualified node name instead of the node basename. |
| `health_publish_period_sec` | `double` | `1.0` | Typed snapshot/service-cache refresh period; must be in `[0.1, 1.0]` seconds. |
| `service_request_timeout_sec` | `double` | `2.0` | Positive finite steady deadline for controller-manager and coordinator polling requests; expired pending requests are removed before polling retries. |
| `startup_grace_sec` | `double` | `10.0` | Non-negative steady grace for dependencies that have not produced their first observation. Grace changes severity, never makes `motion_ready` true without required evidence. |
| `require_sim_clock` | `bool` | `false` | When true, progressing `/clock` samples are required; a backward jump is counted as a reset and fresh progress. |
| `joint_states_topic` | `string` | `/joint_states` | `sensor_msgs/msg/JointState` source monitored with sensor-data QoS. |
| `controller_manager_service` | `string` | `/controller_manager/list_controllers` | Controller-state polling endpoint. Required controllers must report `active`. |
| `planning_scene_state_service` | `string` | `/planning_scene_coordinator/get_state` | Coordinator-state polling endpoint used for backend, revision, reconciliation, lease-expiry and capacity health. |
| `motion_backend_action` | `string` | empty | Required deployed motion action name. Empty or unavailable configuration fails readiness closed. |
| `motion_backend_action_type` | `string` | empty | Required fully qualified action type used by the generic motion-backend probe. |
| `gripper_action` | `string` | empty | Required `control_msgs/action/GripperCommand` endpoint; empty or unavailable fails readiness closed. |
| `base_frame` | `string` | empty | Robot base TF frame. It and `end_effector_frame` must be non-empty and distinct. |
| `end_effector_frame` | `string` | empty | End-effector TF frame queried from `base_frame` at latest available time. |
| `required_joint_names` | `string[]` | empty | Unique non-empty required-name set for the latest JointState; extra observed joints are allowed, while an empty/invalid profile fails readiness closed. |
| `required_controllers` | `string[]` | empty | Unique non-empty required-name set whose members must be `active`; extra controllers are allowed, while an empty/invalid profile fails readiness closed. |
| `joint_stamp_tolerance_sec` | `double` | `2.0` | Non-negative absolute tolerance between the JointState ROS stamp and the current ROS clock. A zero stamp is reported separately as missing. |
| `planning_scene_operation_stuck_after_sec` | `double` | `30.0` | Positive finite steady age after which one continuously reported coordinator operation is classified as stuck. |

Malformed JointState or simulation-clock stamps (negative seconds or
unnormalized nanoseconds) are ignored before updating cached data or freshness.
A subsequent valid sample resumes normal observation. Zero stamps retain their
existing meaning: missing joint stamp or simulation-clock epoch zero.

Freshness thresholds are finite non-negative seconds and must satisfy
`warn_after <= stale_after`. They use receipt/response steady time, not message
ROS stamps:

| Observation | WARN parameter/default | STALE parameter/default |
| --- | --- | --- |
| JointState receipt | `joint_states_warn_after_sec = 0.5` | `joint_states_stale_after_sec = 2.0` |
| Progressing simulation clock | `clock_warn_after_sec = 0.5` | `clock_stale_after_sec = 2.0` |
| Successful controller response | `controllers_warn_after_sec = 2.0` | `controllers_stale_after_sec = 5.0` |
| Successful coordinator-state response | `planning_scene_warn_after_sec = 2.0` | `planning_scene_stale_after_sec = 5.0` |

Perception diagnostic bindings:

Their first-observation state uses the same `startup_grace_sec` as the core
dependencies; later freshness uses the thresholds below.

| Parameter | Type | Default | Semantics |
| --- | --- | --- | --- |
| `perception_diagnostics.topic` | `string` | `/diagnostics` | Non-empty reliable `DiagnosticArray` source. No subscription is created when no pipelines are bound. |
| `perception_diagnostics.allowed_tasks` | `string[]` | empty | Stable task identity for each binding. Repeating one task is allowed for wire-name aliases only when its component ID stays the same. |
| `perception_diagnostics.component_ids` | `int64[]` | empty | Parallel component IDs: stereo `8`, detection `9`, or pose estimation `10`. |
| `perception_diagnostics.status_names` | `string[]` | empty | Parallel exact `DiagnosticStatus.name` allowlist; names and task-to-name pairs must be unique and non-empty. |
| `perception_diagnostics.required_tasks` | `string[]` | empty | Unique stable tasks that block motion readiness; every entry must have a configured binding. |
| `perception_diagnostics.warn_after_sec` | `double` | `2.0` | Non-negative steady age at which a bound diagnostic becomes delayed. |
| `perception_diagnostics.stale_after_sec` | `double` | `5.0` | Non-negative steady age at which it becomes stale; must not be less than the WARN threshold. |

`allowed_tasks`, `component_ids`, and `status_names` must have equal lengths.
Only allowlisted exact status names refresh a pipeline. Diagnostic levels map
to the typed health level, while receipt age is tracked independently; ROS
timestamps in `DiagnosticArray` do not drive freshness. After aliases of the
same task are collapsed, each perception component ID may have only one stable
task; mapping distinct tasks to the same component fails health aggregation.

Fault-injection parameters are `bool`, default to `false`, and are intended for
tests rather than deployment policy:

| Parameter | Forced evidence |
| --- | --- |
| `fault_injection.force_joint_states_stale` | Joint-state freshness is STALE. |
| `fault_injection.force_clock_stale` | Simulation-clock freshness is STALE. |
| `fault_injection.force_controllers_unavailable` | Controller freshness is STALE. |
| `fault_injection.force_tf_unavailable` | Robot TF lookup is unavailable. |
| `fault_injection.force_motion_backend_unavailable` | Motion action is unavailable. |
| `fault_injection.force_gripper_unavailable` | Gripper action is unavailable. |
| `fault_injection.force_planning_scene_unavailable` | Coordinator service/state evidence is unavailable and stale. |
| `fault_injection.force_planning_scene_backend_unavailable` | Coordinator backend-ready flag is false. |
| `fault_injection.force_planning_scene_backend_error` | Coordinator backend error is `BACKEND_FAILED`. |
| `fault_injection.force_planning_scene_reconciliation_required` | Coordinator reconciliation-required flag is true. |
| `fault_injection.force_planning_scene_lease_expired` | Unreconciled lease-expiry count is non-zero. |
| `fault_injection.force_planning_scene_capacity_exhausted` | Operation-ledger capacity is exhausted. |
| `fault_injection.force_perception_unavailable` | Every configured perception pipeline is STALE. |

The exact robot and backend bindings are supplied by a configuration profile.
An absent required stream, service, TF, action, or diagnostic status fails
readiness closed and is reported with a typed issue code.
