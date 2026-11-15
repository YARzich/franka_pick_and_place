# cuRobo PlannerManager API

## Public integration surface

- plugin id: `manipulation_curobo/CuRoboPlanner`
- MoveIt goals: one complete joint constraint set, or one position plus one
  orientation constraint for the configured end-effector link
- side effect: returns a plan-only `RobotTrajectory`

There is no package-specific ROS action, node executable, controller interface,
scene-writing API, installed C++ header, helper library or exported CMake
target. Pluginlib is the only integration surface.

## MoveIt pipeline parameters

Parameters live below the selected planning-pipeline namespace.

| Name | Default | Meaning |
| --- | --- | --- |
| `python_executable` | `python3` | Exact argv executable; no shell is used. |
| `bridge_script` | installed one-shot bridge | Private request/result process entrypoint. |
| `motion_gen_config` | required | Robot-owned MotionGen YAML path or safe `package://` resource. |
| `ee_link` | required | cuRobo pose target link. |
| `max_concurrent_plans` | `1` | Non-blocking process admission capacity. |

The MotionGen YAML owns robot and numeric backend settings, but must not contain
`world_cfg`. Additional integration-only keys are removed before calling the
cuRobo builder:

| `motion_gen` key | Meaning |
| --- | --- |
| `workspace_frame` | Required cuRobo base frame; must match every request and scene frame, including joint goals. |
| `attached_link_map` | Mapping from MoveIt attachment link to cuRobo proxy link. |
| `attached_touch_links` | Per MoveIt link, the touch-link set confirmed by the robot profile/self-collision config. |

Optional `collision_checker` accepts `MESH`, `VOXEL` or
`SIGNED_DISTANCE_FIELD` case-insensitively. Unknown non-empty values fail
configuration/readiness instead of silently selecting another checker. An
absent or empty value leaves the backend default unchanged; a known checker
missing from the selected cuRobo version also fails explicitly.

Robot configs with attached geometry must reserve collision spheres on every
proxy link. The current MotionGen API resolves external geometry relative to
its configured end effector, so the MoveIt attachment link must equal `ee_link`
and the proxy must be fixed to it. Any other mapping fails rather than
approximating a moving object as a static obstacle.

The bridge loads a fresh robot config for every request and replaces its
`self_collision_ignore` map with the exact enabled robot-link pairs read from
the request ACM plus the configured proxy/touch pairs. Consequently, a stock
cuRobo robot file with a different ignore table cannot silently weaken the
MoveIt ACM, and the backend profile does not duplicate scene policy.

## Private file schema

The version-2 YAML transport is an implementation detail, not a ROS contract.
One request contains:

- exact group joint names/start positions, goal, timeout and scaling;
- `scene.is_diff=false` and `scene.robot_state.is_diff=false`;
- planning frame plus resolved planning-frame transforms;
- all world and attached collision object parts with stable names and owner ids;
- complete AllowedCollisionMatrix;
- robot-owned MotionGen config path.

Each child receives private temporary paths. A response contains success,
planning time and trajectory, or one typed category: `planning_failed`,
`timed_out`, `invalid_request`, `unsupported_scene`, or `internal_error`.

The response boundary uses the complete resolved start `RobotState`. It accepts
the first trajectory point only within `1e-4` per group variable, checks every
waypoint against joint bounds and the immutable PlanningScene, and preserves
all non-group variables when constructing `RobotTrajectory`.

## Startup and planning budgets

Plugin initialization runs a 30-second-bounded, side-effect-free probe. It
executes the selected interpreter and bridge, imports required modules, parses
MotionGen and robot configs, and checks CUDA availability. It does not
instantiate or warm MotionGen or download resources. A failed explicit
selection therefore prevents the pipeline from becoming available before task
goal admission.

`allowed_planning_time` becomes `MotionGenPlanConfig.timeout` and measures only
the `plan_single*` call. Imports, MotionGen construction and warmup are outside
that API budget. The request child has no duplicate C++ planning deadline, but
its wait is interruptible by cancellation and shutdown.

The supervisor owns the entire worker process group until cleanup finishes.
Leader exit does not end that ownership: cleanup allows the leader bounded
SIGINT/SIGTERM grace, then sends SIGKILL to remaining group members before
reaping the leader. This also applies to normal leader exit. Waiting with
`WNOWAIT` reserves the leader's identity until the last signal, preventing PID
reuse from redirecting cleanup to another group. Its original exit status is
preserved. Descendants must stay in the inherited process group.

## MoveIt error mapping

| Backend condition | MoveIt code |
| --- | --- |
| planning success | `SUCCESS` |
| no trajectory | `PLANNING_FAILED` |
| planning budget exhausted | `TIMED_OUT` |
| exact-goal cancellation | `PREEMPTED` |
| invalid start state | `INVALID_ROBOT_STATE` |
| invalid goal/request | `INVALID_GOAL_CONSTRAINTS` |
| scene not exactly representable | `COLLISION_CHECKING_UNAVAILABLE` |
| bridge process unavailable | `COMMUNICATION_FAILURE` |
| malformed/internal response | `FAILURE` |

Only the ordinary MoveIt pipeline selection changes when choosing this backend;
strategy, task client, physical execution, gripper, scene coordinator and the
high-level action endpoint remain unchanged.
