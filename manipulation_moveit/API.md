<!-- Responsibility: Define the public ROS, parameter, plugin, and C++ contracts. -->
# API

## ServoToPose server

Executable: `moveit_servo_task_server`

This executable hosts only the reactive pose task. It always loads a complete
Servo profile and exposes no object-transfer action or scene/gripper ports.

| Interface | Default | Type |
| --- | --- | --- |
| Action | `~/servo_to_pose` | `manipulation_msgs/action/ServoToPose` |
| Pause service | `~/servo_to_pose/set_task_paused` | `manipulation_msgs/srv/SetTaskPaused` |

The goal contains one validated `geometry_msgs/msg/PoseStamped`: a canonical
relative frame, normalized non-zero stamp, finite pose, and unit quaternion. A
zero goal timeout selects the configured default. A positive timeout must leave
the task's bounded safe-stop reserve. Feedback reports the standard task state
and fresh frame-consistent position/orientation error. The terminal
`MotionTaskResult` carries a typed error, steady elapsed duration, and
`stop_confirmed` evidence.

Required identity and robot binding:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `servo_to_pose.preset_id` | `servo_to_pose/default` | Canonical immutable preset family ID. |
| `servo_to_pose.preset_schema_version` | `1` | Positive preset schema version. |
| `servo_to_pose.arm_resource_id` | `arm/main` | Logical arm resource claimed by the action. |
| `servo_to_pose.planning_frame` | required | Frame used for current error and tip target. |
| `servo_to_pose.task_frame` | required | Controlled frame named by the task contract. |
| `servo_to_pose.servo_tip_frame` | required | Command frame expected by MoveIt Servo. |
| `servo_to_pose.joint_names` | required | Unique controlled joints whose velocities prove stop. |

Task/action policy:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `servo_to_pose.action_name` | `~/servo_to_pose` | Public action endpoint. |
| `servo_to_pose.pause_service_name` | `~/servo_to_pose/set_task_paused` | Cooperative pause/resume endpoint. |
| `servo_to_pose.position_tolerance_m` | `0.001` | Linear completion tolerance. |
| `servo_to_pose.orientation_tolerance_rad` | `0.01` | Angular completion tolerance. |
| `servo_to_pose.stopped_velocity_tolerance` | `0.01` | Maximum absolute joint velocity accepted as stopped. |
| `servo_to_pose.required_target_samples` | `3` | Consecutive distinct in-tolerance pose samples. |
| `servo_to_pose.required_stopped_samples` | `3` | Consecutive distinct low-velocity samples. |
| `servo_to_pose.halt_timeout_sec` | `2.0` | Safe-stop confirmation bound. |
| `servo_to_pose.control_period_sec` | `0.02` | Task-loop period. |
| `servo_to_pose.incoming_command_timeout_sec` | `0.1` | Target-stream freshness budget. |
| `servo_to_pose.service_call_timeout_sec` | `0.5` | Bound for one backend service operation. |
| `servo_to_pose.target_prepare_timeout_sec` | `1.0` | Exact-stamp target preparation bound. |
| `servo_to_pose.feedback_startup_timeout_sec` | `0.5` | Initial backend/TF/joint feedback bound. |
| `servo_to_pose.observation_timeout_sec` | `0.005` | Bound for one observation read. |
| `servo_to_pose.heartbeat_period_sec` | `0.025` | Action feedback heartbeat. |
| `servo_to_pose.default_timeout_sec` | `30.0` | Goal deadline selected by a zero request timeout. |

MoveIt Servo ROS bindings:

| Parameter | Default | Stable ROS interface |
| --- | --- | --- |
| `servo_to_pose.pose_command_topic` | `/servo_node/pose_target_cmds` | `PoseStamped` target stream. |
| `servo_to_pose.command_type_service` | `/servo_node/switch_command_type` | Select pose command mode. |
| `servo_to_pose.backend_pause_service` | `/servo_node/pause_servo` | Pause/resume the backend. |
| `servo_to_pose.status_topic` | `/servo_node/status` | `moveit_msgs/msg/ServoStatus`. |
| `servo_to_pose.joint_state_topic` | `/joint_states` | Controlled-joint feedback. |
| `servo_to_pose.poll_period_sec` | `0.005` | Internal synchronous ROS wait slice. |
| `servo_to_pose.current_tf_max_age_sec` | `0.25` | Maximum current-pose TF age. |
| `servo_to_pose.joint_state_max_age_sec` | `0.25` | Maximum local and stamped joint-state age. |
| `servo_to_pose.status_max_age_sec` | `0.5` | Maximum local Servo-status age. |

`load_moveit_servo_process_config()` validates the complete snapshot before the
host creates ROS entities. There is no `servo_to_pose.enabled` parameter in the
dedicated process. Invalid frames, logical IDs, endpoints, sample counts, or
timing terminate startup.

Goal admission remains closed until the command-mode and pause services, a pose
command subscriber, fresh Servo status, fresh controlled-joint velocities, and
a non-zero ROS clock are present. Every terminal path stops target publication
and requires both fresh distinct low-velocity evidence and a confirmed paused
backend. Failure to prove stop keeps the arm resource quarantined.

## V2 C++ backend

Headers are under `manipulation_moveit/`; implementations use namespace
`manipulation::moveit`.

### Plan-only arm boundary

Link `manipulation_moveit::arm_planning` for `ArmPlanningPort` and
`MoveItArmPlanningAdapter`. `ArmPlanningRequest` value-owns a complete
authoritative `moveit_msgs/msg/PlanningScene` and its coordinator
session/revision proof. Both scene and robot state must be non-diff.

`PlanningOutcome` separates retryable, non-retryable, and stopped results. Only
MoveIt's internal budget exhaustion is retried, using bounded stop-aware
backoff. `IPlanningSceneRevisionHook` must confirm the same session/revision
before each attempt. The adapter sends `plan_only=true` with a private fixed
MoveIt attempt budget; it has no trajectory-execution, gripper, or mutation
endpoint.

`ArmPlanningPort::plan(request, execution_context)` is the production entry.
MoveGroup discovery and goal-response/result waits have no transport deadline
and periodically call the execution heartbeat. Retry backoff is sliced through
cooperative checkpoints. The final `async_send_goal` is executed only inside
the lifecycle operation gate, which atomically orders it against pause,
cancel, and shutdown. A pause therefore prevents a new planner request but
does not discard a request already in flight.

The full-scene transport requires the MoveGroup capability
`manipulation_moveit/ExactPlanningSceneMoveGroupCapability`. In the V2
MoveGroup process, add that plugin to `capabilities` and add the upstream
`move_group/MoveGroupMoveAction` to `disable_capabilities`; both serve the same
canonical `move_action`, so they must never be enabled together. The exact
capability accepts only plan-only requests with non-diff scene and robot state,
builds a new request-owned `PlanningScene`, and then invokes the selected
MoveIt `PlanningPipeline`. OMPL and cuRobo therefore retain their normal
PlannerManager selection while backend-only stale world objects cannot leak
into the request.
Cancellation is owned by the accepted goal UUID. While `generatePlan()` owns
its worker, a private cancellation watcher retries `PlanningPipeline::terminate()`
after cancel, including the request-adapter interval before a planning context
exists. The worker joins that watcher before terminal completion/admission of
another goal. Late cancel callbacks cannot affect a subsequent goal. No
whole-goal deadline is added; an adapter must return, and a planner must honor
its termination contract, before the worker can finish.

Immediately before every planning request, the strategy refreshes the full
scene and robot state through its coordinated lease. A successful trajectory
is accepted only when its first joint and multi-DOF waypoint matches that
snapshot; missing, malformed, non-finite, or mismatched start evidence is a
non-retryable planning failure.

The split planning/execution handoff adds two runtime guards. After a successful
trajectory, the next planning input waits for fresh `/joint_states` evidence
that the commanded endpoint is quiescent. Immediately before
`ExecuteTrajectory`, the strategy revalidates the full scene and compares the
planned first waypoint with its refreshed robot state. A valid mismatch
discards the stale trajectory and replans from that snapshot; malformed
evidence fails closed without dispatching a physical goal.
Quiescence samples require non-zero normalized stamps in the current ROS time
epoch, no future stamp, and at most 250 ms of ROS age at processing. Replayed or
stale samples reset the convergence evidence and wait for fresh samples within
the accepted command budget. A missing clock, backwards clock jump, or malformed
stamp fails closed; after controller success this leaves physical readiness
quarantined because terminal quiescence was not proven.
Numeric convergence, replay and clock-epoch rules live in a private policy;
the ROS adapter owns message validation, sampling, action transport and waits.

### Pick/lift/place strategy

Link `manipulation_moveit::pick_lift_place_strategy` for
`PickLiftPlaceStrategy`, stable strategy ID `pick_lift_place`. This is a direct
C++ `TransferExecutionStrategy`; no lower ROS task action is involved.

`PickLiftPlaceStrategyConfig` contains robot binding/open/home values, lift,
preplace, retreat, release settling, and exactly two physical bounds:
`arm_motion_timeout` and `gripper_command_timeout`. Candidate pregrasp/grasp
geometry drives approach. Planning, readiness, and scene convergence have no
task timeout.

Before every physical or scene side effect the strategy performs a cooperative
checkpoint, waits for health readiness, and revalidates the exact full-scene
lease. Attach follows only a confirmed grasp; detach follows only confirmed
place and release evidence. Recovery preserves the best-known object
disposition and typed resource-release evidence.
The strategy records confirmed gripper postconditions before observing cancel
at the next checkpoint. An accepted close/release with unknown postcondition
keeps the object disposition unknown. A confirmed release without confirmed
scene detach reports scene uncertainty and retains the scene lease, while
preserving terminal-safe physical resource evidence; it performs no motion
recovery against the unresolved attached scene object.
A private typed gripper effect distinguishes preparation, grasp and release;
diagnostic labels have no role in object disposition or scene-lease decisions.

### Production physical and health adapters

Link `manipulation_moveit::pick_lift_place_ros_adapters`:

- `RosTrajectoryExecutionPort` maps
  `moveit_msgs/action/ExecuteTrajectory`;
- `RosStrategyGripperPort` maps
  `control_msgs/action/GripperCommand`, including `reached_goal` and an
  explicitly allowed close-stall;
- `RosPickLiftPlaceHealthPort` combines reliable transient-local
  `SystemHealthSnapshot` with injected planning, trajectory, gripper, and scene
  readiness probes.

The position-only close contract uses the neutral command
`position=0.0, max_effort=0.0`; it does not pretend that candidate opening or
force metadata can be enforced by a position controller. Allowed stall and the
typed postcondition remain the evidence for a confirmed hold.

The physical configs contain endpoints only. Discovery and goal acceptance do
not consume the supplied motion budget; its clock starts in the accepted
goal-response callback. Stop targets the exact accepted goal. Late callbacks
remain owned by a generation-safe active slot, and an unconfirmed terminal
state produces `kActiveStateUnknown` rather than an unsafe resource release.

The maintained composition replaces upstream
`move_group/MoveGroupExecuteTrajectoryAction` with
`manipulation_moveit/CancelableExecuteTrajectoryCapability`: add the latter to
MoveGroup `capabilities` and the former to `disable_capabilities`. Both use the
unchanged `execute_trajectory` endpoint and `moveit_msgs/action/ExecuteTrajectory`;
only one may be enabled. The replacement addresses the installed MoveIt 2.12.4
capability whose blocking accepted callback prevents timely cancellation and
whose cancel callback does not stop the trajectory manager.

The local capability reserves one UUID, rejects overlapping requests, and
keeps ROS callbacks responsive while one owned worker dispatches, stops, and
joins the TrajectoryExecutionManager. Exact cancel before dispatch sends no
trajectory; cancel after dispatch stops and joins before publishing terminal
evidence. Shutdown closes admission and joins the worker. An uncertain
dispatch/stop exception retains the unresolved goal handle and rejects further
goals until restart, allowing the caller to preserve physical quarantine.
Shutdown disables the action server before retiring unresolved handles and
retries worker-owned stop/join; an unconfirmed stop remains a logged fault,
never a terminal-safe result. This
capability does not plan, write PlanningScene, or implement workflow recovery.

Discovery and the short `async_send_goal` boundary use cooperative checkpoints
and the lifecycle operation gate. After dispatch, polling uses heartbeat
without applying pause: this keeps the goal-scoped scene lease renewable even
when an accepted command budget exceeds the 45-second lease TTL. Cooperative
pause is observed at the next strategy checkpoint after exact-goal terminal
evidence; cancel still targets the exact active goal immediately.

The health config contains only its topic. Freshness and wait slices are private
invariants. `wait_until_ready()` has no semantic deadline and ends only on
readiness, typed final error, cooperative cancel, or shutdown.

### Coordinated scene adapter

Link `manipulation_moveit::v2_scene_adapter` for `RosV2SceneAdapter`. It is the
single production scene boundary shared by resolution, planning, and execution.
It implements `TransferSceneWorkflowPort`, `IPickLiftPlaceScenePort`, and
`IPlanningSceneRevisionHook`.

`RosV2SceneAdapterConfig` contains only owner/link/resource identity and ROS
service endpoints. Full snapshots are accepted only when coordinator state is
stable before and after `GetPlanningScene` and lease validation proves the same
session/revision. Authoritative mutation is sent only through the coordinator's
apply-diff service. Endpoint/response waits end on response, typed final error,
cancel, or shutdown; lease TTL and wait slices are private.

All goal-scoped `async_send_request` calls are linearized through the normal
pause/cancel operation gate; a stopped or paused gate cannot leak a request.
ROS context validity is checked before endpoint readiness and again inside that
gate, so a ready endpoint cannot receive a request after observed shutdown.
`RenewPlanningSceneLease` is the sole maintenance exception: its transport send
uses the stop/deadline-only maintenance gate and its response wait uses
heartbeat, allowing renewal during pause and accepted physical execution.
Post-stop lease release remains an explicit best-effort cleanup rather than a
new goal side effect.

`GeometricShapesMeshResourceLoader` supplies package/resource mesh loading for
catalog conversion. Primitive, mesh, and compound geometry remain owned by the
shared object catalog/domain types.

## Octomap gate

Cloud timestamps are validated before `tf2_ros::MessageFilter`: negative seconds
and non-normalized nanoseconds are rejected and counted in
`invalid_stamp_rejections`; the existing `zero_stamp_rejections` remains separate.
A rejected cloud does not stop the gate or prevent a later valid cloud from passing.

Executable: `moveit_octomap_tf_gate`

| Interface | Default | Type |
| --- | --- | --- |
| Input | `~/input` | `sensor_msgs/msg/PointCloud2` |
| Output | `~/output` | `sensor_msgs/msg/PointCloud2` |
| Diagnostics | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

`expected_source_frame` and `required_frames` are required. `queue_size`,
`transform_timeout_sec`, and `health_timeout_sec` are positive safety bounds. A
zero or malformed stamp, unexpected source frame, missing exact-stamp TF, duplicate required
frame, or empty frame fails closed. The forwarded cloud is byte-for-byte
unchanged so its physical sensor origin remains valid for ray casting.

The node implementation is isolated in
`manipulation_moveit::octomap_tf_gate_support`; it no longer depends on any task
backend target.

Plugin class:
`manipulation_moveit/FailClosedPointCloudOctomapUpdater`. It extends MoveIt's
point-cloud updater and accepts a self-filter pass only when its transform cache
contains exactly every active exclusion shape.
