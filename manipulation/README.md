<!-- Responsibility: Explain the reusable manipulation runtime and its ownership boundaries. -->
# manipulation

Robot-agnostic C++17 runtime for long-running manipulation tasks. The package
owns task state, cooperative cancellation and pause, a single-writer
PlanningScene coordinator, generic action lifecycle, and health aggregation.
Robot descriptions, controller names, joint sets, task poses, object models,
simulator worlds, detector choices, and concrete scenario requests belong to
an application package.

The package deliberately keeps several exported libraries instead of hiding
the implementation behind one monolithic target:

| Target | Responsibility |
| --- | --- |
| `manipulation::transport` | Header-only bounded ROS service-call transport with pending-request cleanup. |
| `manipulation::task` | ROS-independent task status and execution control. |
| `manipulation::servo` | Pure deterministic ServoToPose state machine: tracking evidence, pause/cancel, bounded safe stop, and terminal outcome. |
| `manipulation::skill` | Immutable skill descriptors, capability checks, preset identity, and process-local resource ownership. |
| `manipulation::world` | Immutable backend-neutral snapshots of stamped rigid-object state. |
| `manipulation::recording` | Nonblocking lifecycle-event boundary for optional episode recording. |
| `manipulation::scene_registry` | Exclusive leases, revisions, quarantine, and idempotency. |
| `manipulation::scene_client` | Client-side revision and retry policy. |
| `manipulation::scene_support` | PlanningScene validation, canonicalization, and frame adaptation. |
| `manipulation::workflow` | Action lifecycle plus ROS-independent ordered-sequence policy, pause, cancel, feedback, and worker ownership. |
| `manipulation::monitoring` | Pure freshness and health classification policy. |

`manipulation_moveit` may depend on these targets to implement planning and
execution ports. This package must never depend on a planner backend or robot
application. A reference backend follows the same inward-only dependency rule.

## Reactive Servo Policy

`manipulation::servo` contains only the backend-neutral policy for moving
towards one fixed pose. `ServoToPoseStateMachine` receives explicit pose,
velocity, backend-state, safety, pause, cancel, and deadline observations. Each
call returns at most one declarative command: publish the fixed target, request
resume, request pause, or do nothing.

The state machine owns no ROS node, topic, service, TF buffer, MoveIt object,
thread, or robot configuration. This makes its completion and stop behavior
fast to test and reusable with another reactive controller. A backend adapter
is responsible for obtaining fresh observations and executing the returned
commands.

Normal completion and every interruption use the same safe-stop invariant:
the target stream stops, fresh uniquely numbered low-velocity samples prove
that motion has stopped, and the backend is confirmed paused. Normal success
additionally requires new pose samples after that pause to prove that the
stopped TCP remains inside tolerance. While a running backend is being stopped,
the pause command is issued only after low velocity has been observed. Repeated
reads of one cached sample cannot satisfy any required sample count.

This policy is deliberately independent from the planned transfer workflow.
Servo motion does not plan around a complete scene and does not replace planned
motion; an application chooses which separate skill is appropriate for a task.

## Skill Runtime Contract

A long-running operation can describe itself with a `SkillDescriptor` instead
of hiding its deployment requirements in one entrypoint. The descriptor keeps
four independent facts:

- a stable skill type such as `transfer/object`;
- an immutable preset identity: readable ID, schema version, and SHA-256 digest
  of the exact canonical configuration;
- the robot-independent capabilities required by the skill;
- the logical command resources that must remain exclusively owned while one
  execution is active.

`SkillRuntime` validates required capabilities when the process is composed and
atomically acquires all declared resources before work starts. Its move-only
lease is released after terminal delivery only when the backend confirms that
no physical command remains unresolved. Otherwise the host retains the exact
lease until restart, so a new skill cannot overlap an arm that may still move.
Resource IDs such as `arm/main` and `gripper/main` describe physical command
ownership, not ROS endpoint names.
The registry is deliberately process-local and nonblocking: applications that
host multiple skill facades must inject one shared registry, while arbitration
between different processes remains a deployment responsibility.

The same runtime can emit best-effort `EpisodeEvent` values at lifecycle
boundaries. The default sink discards them. A custom sink may only enqueue into
bounded owned storage; disk, network, or dataset work must happen elsewhere.
Recording failure or backpressure never changes task execution.

## World-State Contract

`WorldStateSnapshot` is an immutable value for the exact stamped rigid-object
state selected by an execution. It carries source, session, revision, capture
stamp, an opaque correlation token, and sorted unique object poses with
optional confidence. It intentionally carries no planner, robot model,
Octomap, image, or object-shape type.

A backend decides how much of the world it can prove. The current MoveIt
transfer backend records the authoritative transferred object read from
PlanningScene and identifies it with that coordinator session and revision.
Episode events refer to the snapshot by token, so metadata can be correlated
without copying sensor or scene payloads into feedback.

## Scene Contract

PlanningScene is the shared collision-scene representation for the current
architecture. The coordinator is robot-agnostic, but intentionally uses
`moveit_msgs` as its stable scene transport. A backend that does not plan with
MoveIt may still mirror this authoritative scene through an adapter.

The installed coordinator profile uses `world` for both public and backend
frames. Robot applications normally provide their own parameter file and set
`planning_frame`, `backend_planning_frame`, and `robot_description_topic`
explicitly.

Scene leases model implicit MoveIt side effects: an object's world and attached
keys share one lifecycle scope, and that scope excludes foreign ACM leases that
name the object. A workflow acquires its world, attached, and touch-link ACM
keys atomically. This keeps object removal from silently deleting collision
policy owned by another workflow while preserving independent ownership for
unrelated objects and ACM pairs.

```bash
ros2 launch manipulation planning_scene_coordinator.launch.py \
  config_file:=/absolute/path/to/robot_scene.yaml \
  use_sim_time:=true
```

## Health Contract

The monitor publishes one transient-local typed snapshot and standard
diagnostics. Robot-specific joint, controller, frame, gripper, and backend
action bindings are parameters. Missing bindings remain visible and block
motion admission rather than selecting a hidden robot default.

## V2 Transfer Workflow

`manipulation::workflow` exposes one versioned high-level facade,
`TransferObjectSequenceV2ActionServer`. Its goal carries object intent rather
than resolved source geometry or backend details. The facade owns one active
goal, cooperative pause, exact-goal cancellation, managed worker lifetime,
typed feedback, and reliable terminal delivery. It owns no planner, controller,
gripper, TF buffer, detector, or robot model.

`ResolvedTransferWorkflow` resolves each accepted step through an injected
`ObjectCatalog`, `ObservationProvider`, `GraspProvider`, coordinated scene port,
and selected `TransferExecutionStrategy`. It keeps the immutable scene
session/revision evidence with the resolved task, stops at the first failed
step, preserves per-step outcomes, and performs explicit goal-scoped cleanup.
Configured and perception-backed applications differ only in the providers
chosen by their composition root.

An object profile may declare physically equivalent proper rotations in its
local frame. The workflow uses them only when resolving
`TARGET_INITIAL_SOURCE`: it chooses the recorded source orientation equivalent
that is nearest to the current observation. Raw observations, scene staging,
grasp input, and explicit absolute targets are not rewritten.

The goal-scoped scene lease is fail-closed at termination. A normal RAII
destructor does not release it until the workflow explicitly finalizes terminal
evidence. Confirmed safe-idle, source, or target dispositions release only when
every accepted physical command also reached a release-safe boundary.
`HOLDING_OBJECT`, unknown or scene-uncertain state, an ambiguous dispatched
scene mutation, and `kQuarantineRequired` retain the lease for coordinator TTL
or explicit recovery. Cancellation before strategy dispatch still releases
when no ambiguous scene mutation occurred.

The neutral sequence lifecycle is shared by the V2 facade without exposing the
generated action type to its core. `TransferSequenceSession` linearizes pause,
stop, backend operations, and completion. `TransferSequenceGoalReservation`
provides a nonblocking process-local resource gate, while
`TransferSequenceExecutionPolicy` validates rich backend evidence before it is
delivered on ROS. None of these contracts introduces a task-configurable
whole-goal timeout for V2.

Short downstream dispatches pass through the same lifecycle operation gate, so
cancel or pause cannot race between a safe checkpoint and command submission.
Already accepted physical commands use a stop-only heartbeat: pause waits for
the next safe boundary while scene-lease renewal and throttled wait diagnostics
continue. A separate stop-atomic maintenance gate lets lease renewal proceed
during pause without weakening cancellation for ordinary side effects.

Robot applications own provider profiles, concrete backend ports, physical
command deadlines, endpoint names, and the process composition. This package
installs only the generic libraries plus the PlanningScene coordinator and
health-monitor infrastructure executables; it no longer installs static-box or
lower-level transfer action clients.

## Source Layout

Public headers follow the same responsibility split:

```text
include/manipulation/task/
include/manipulation/servo/
include/manipulation/skill/
include/manipulation/world/
include/manipulation/recording/
include/manipulation/transport/
include/manipulation/scene/
include/manipulation/workflow/
include/manipulation/monitoring/
```

Fast tests remain beside their owning package. Architecture tests reject
dependencies on robot applications, concrete planner packages, and
scenario-specific entrypoints.
