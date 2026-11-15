<!-- Responsibility: Explain the manipulation_moveit package boundary, setup, and verification. -->
# manipulation_moveit

Robot-independent MoveIt adapters for the V2 manipulation workflow, a
standalone reactive `ServoToPose` task, and fail-closed Octomap input. Robot
descriptions, controller names, grasp generation, object catalogs, task
composition, and simulation worlds belong to a robot-specific application.

The package no longer hosts a lower task-level object-transfer action. Planned
transfer is assembled by the application from direct C++ strategy/backend
ports, so task lifecycle and physical side effects remain owned by the single
V2 workflow server.

## Libraries

The exported libraries have narrow responsibilities:

- `arm_planning` provides `ArmPlanningPort` and the plan-only
  `MoveItArmPlanningAdapter`;
- `pick_lift_place_strategy` implements the direct C++ `pick_lift_place`
  strategy over borrowed planning, execution, gripper, health, and scene ports;
- `pick_lift_place_ros_adapters` maps ExecuteTrajectory, GripperCommand, and
  typed system-health ROS contracts to those ports;
- `v2_scene_adapter` reads a complete authoritative PlanningScene and performs
  coordinator-owned V2 scene mutations;
- `moveit_servo_ros_adapter` and `moveit_servo_task` implement the independent
  reactive pose task;
- `moveit_servo_task_host` is the standalone Servo action composition root;
- `octomap_tf_gate_support` and `fail_closed_octomap_updater` implement the two
  fail-closed occupancy-map safeguards.

`ArmPlanningRequest` value-owns a complete non-diff PlanningScene plus its
coordinator session/revision proof. The MoveIt adapter is plan-only: it never
executes a trajectory, operates a gripper, or mutates the scene. Only an
internal MoveIt budget exhaustion is retryable, under bounded stop-aware
backoff and a fresh revision check. MoveGroup discovery, goal response, result,
and retry waits service the goal heartbeat, so scene ownership and wait
diagnostics stay alive. Goal dispatch is linearized with pause/cancel admission;
pause is applied before a request is sent, never by abandoning an in-flight
planner result.

V2 MoveGroup deployments must disable `move_group/MoveGroupMoveAction` and
enable `manipulation_moveit/ExactPlanningSceneMoveGroupCapability`. The
replacement owns the same canonical `move_action`, creates a new PlanningScene
from each full snapshot, and delegates to the selected MoveIt planning
pipeline, so OMPL/cuRobo selection remains unchanged without inheriting hidden
backend world state. The strategy refreshes scene and robot state immediately
before each plan; the adapter rejects a successful trajectory whose first
waypoint does not match that snapshot.

The split plan/execute path also waits for fresh `/joint_states` to prove
endpoint quiescence before constructing the next plan. After planning it
refreshes the full scene once more and compares that robot state with the first
waypoint; drift discards the trajectory and replans before any
ExecuteTrajectory goal is sent.

`PickLiftPlaceStrategy` takes pregrasp/grasp geometry from the selected grasp
candidate. Before every physical command and attach/detach mutation it performs
a cooperative checkpoint, waits for typed readiness, and revalidates the exact
scene lease. Controller budgets begin only after the controller accepts the
specific goal and include the terminal `/joint_states` convergence check after
a successful ExecuteTrajectory result. Discovery and dispatch are pause/cancel
gated. Once a physical goal may be active, heartbeat continues to renew the
scene lease while pause is deferred to the next proven-safe strategy boundary.
If the controller reports success before the robot settles, the same accepted
budget bounds that final evidence wait; expiry, cancel, or malformed evidence
keeps the command resource quarantined until process restart.
For a position-only gripper, close is encoded as the neutral
`position=0.0, max_effort=0.0`; allowed stall and the typed postcondition prove
the hold without claiming force control.

Every goal-scoped scene service request is dispatched through the same
operation gate, eliminating check-then-send races. Lease renewal alone uses a
stop/deadline-only maintenance gate so ownership can remain valid while the
goal is paused or an accepted physical command is still running.

## ServoToPose runtime

Executable `moveit_servo_task_server` hosts only `ServoToPose`. It constructs no
planned-transfer configuration, ports, or action facade. The server always
loads one complete `servo_to_pose.*` profile; there is no enable switch in this
dedicated process.

The goal is a fixed stamped pose. Target preparation uses exact-stamp TF, and
the action remains closed until its MoveIt Servo services, command consumer,
status, joint feedback, and ROS clock are ready. Normal completion, pause,
cancel, timeout, and backend failures all use the same safe-stop contract:
target publication ends, fresh distinct joint samples prove low velocity, and
the backend is confirmed paused. Without that evidence the arm-resource lease
remains quarantined until process restart.

The process claims the configured logical arm resource through a local
`ResourceRegistry`. This coordinates facades sharing that registry; it is not a
distributed lock between independent processes.

See [API.md](API.md) for parameters and stable ROS/C++ interfaces.

## Octomap safeguards

Executable `moveit_octomap_tf_gate` forwards a stamped point cloud unchanged
only when exact-stamp TF exists to every configured robot leaf frame. The
plugin `manipulation_moveit/FailClosedPointCloudOctomapUpdater` independently
rejects a MoveIt self-filter pass unless its transform cache contains exactly
every active exclusion shape.

The gate requires `expected_source_frame` and `required_frames`. Camera profiles
must preserve the physical sensor frame and a non-zero stamp so Octomap ray
casting and robot self-filtering use the same sample time.

## Package boundary

The package intentionally contains no robot model, world, sensor pipeline,
object defaults, grasp generator, or application composition. Generic code does
not depend on `franka_motion`; an application may depend on and configure these
MoveIt components in the opposite direction.

## Verification

CPU-only tests cover planning classification/retry, exact full-scene replacement,
stale backend-object exclusion, fresh per-plan robot state and start-waypoint fidelity,
pick/lift/place ordering and recovery, cancel/pause-at-dispatch races,
unbounded planning transport and typed shutdown, planning/physical heartbeat
maintenance, delayed physical-goal acceptance, health and scene-service waits
beyond removed five-second limits, cancel/shutdown without late scene requests,
terminal-convergence budget/quarantine, safe-stop versus quarantine,
exact-goal cancellation, ServoToPose state/action
behavior, exact-stamp cloud gating, and the in-process Octomap updater. They do
not prove simulator, GPU, or real-hardware behavior.

```bash
colcon build --packages-select manipulation_moveit \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
colcon test --packages-select manipulation_moveit
colcon test-result --verbose
```
