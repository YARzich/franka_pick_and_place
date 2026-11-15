# manipulation_curobo

Optional robot-independent **experimental backend** for the existing MoveIt
planning contract. Its only public integration point is the
`manipulation_curobo/CuRoboPlanner` PlannerManager plugin. It does not define a
second task action, task lifecycle, execution path, world owner, or robot
profile. Helper libraries and C++ headers are private build details and are not
installed or exported through CMake.

## Recommended path

```text
TransferObjectSequenceV2 workflow
  -> common ArmPlanningPort
  -> MoveGroup request with a complete non-diff PlanningScene
  -> manipulation_curobo/CuRoboPlanner
  -> request-local one-shot Python process
  -> MotionGen plan-only trajectory
```

The planner context snapshots the scene before starting the child process. The
private schema contains the exact planning start, world primitive/mesh/compound
geometry, attached geometry, the AllowedCollisionMatrix, and resolved fixed and
robot-link frame transforms. Python constructs a request-owned `WorldConfig`,
sizes MotionGen from it, and calls `update_world()` before every plan. A static
world in MotionGen configuration is rejected because PlanningScene remains the
single source of truth.

`motion_gen.workspace_frame` is mandatory. Every private bridge request,
including a joint goal, carries that same workspace frame, and its immutable
snapshot `planning_frame` must match it exactly.

## Scene fidelity

- box, sphere and cylinder stay native cuRobo geometry;
- meshes preserve all vertices and triangle indices;
- compound objects become deterministic `<object>__<kind>_<index>` parts while
  retaining their common owner id;
- cones become a closed, circumscribed 32-sided mesh: height and apex are exact,
  radial over-approximation is below 0.5%, so the conversion is conservative;
- attached objects use cuRobo's supported
  `attach_external_objects_to_robot()` API and require an explicit mapping from
  each MoveIt attachment link to a robot-profile proxy link with reserved
  collision spheres;
- full-object ACM allowance is supported. Arbitrary partial link/object ACM
  pairs cannot be expressed by cuRobo and fail closed with a typed unsupported
  scene result. Enabled MoveIt robot self-collision pairs are read directly
  from the request ACM and replace the loaded cuRobo robot config's
  self-collision ignore table. Attached proxy/touch pairs are then added
  explicitly from the robot profile.

Infinite planes, octomaps, prism primitives, unconfigured attachment/touch-link
mappings and partial world ACM are rejected early. In particular, the current
camera/octomap scenario is not a supported cuRobo profile; it must not silently
drop occupancy data.

## Cancellation and process ownership

Every request owns RAII temporary files and one isolated process group. The
manager's admission gate bounds concurrency before process creation. MoveIt
cancel signals and reaps exactly the matching group; steady planning timeout is
reported as `TIMED_OUT`. Invalid request, unsupported scene, planner failure,
backend unavailability and invalid response map to distinct MoveIt error codes.
The bridge plans only and never executes a trajectory or writes PlanningScene.

Selecting the plugin runs a side-effect-free readiness probe during
`PlannerManager::initialize()`, before any task goal can be accepted. It checks
the Python executable, imports, MotionGen/robot YAML, configured frames and CUDA
device availability. It does not build or warm MotionGen, allocate planning
tensors, download assets or start a persistent process.

MoveIt's `allowed_planning_time` is passed only to MotionGen's planning API.
Python imports, MotionGen construction and warmup are not charged to that
budget, while the supervisor wait remains interruptible by exact-request
cancellation and bounded termination during shutdown.

Before success, the adapter checks the first point against the full resolved
MoveIt start state with a `1e-4` joint-position tolerance, validates every
waypoint's bounds and PlanningScene validity, and preserves non-group joints in
the returned trajectory.

## License boundary

This repository's integration code is MIT licensed. The earlier evaluated
cuRobo checkout `ebb71702f3f70e767f40fd8e050674af0288abe8` has a non-commercial
research/evaluation license; the separately installed `nvidia-curobo==0.8.0`
declares Apache-2.0, with separate asset notices. License and API compatibility
must be checked against the selected version: the 0.8.0 license does not prove
that this adapter works with its rewritten API. cuRobo is not vendored,
automatically downloaded, or required by the canonical MoveIt/OMPL path. Review
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) before explicitly enabling
the backend.

## Build and CPU tests

```bash
colcon build --packages-select manipulation_curobo \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
colcon test --packages-select manipulation_curobo
colcon test-result --verbose
```

CPU tests use pure conversion functions, fake MotionGen objects and a one-shot
fake planner runtime. They exercise the real PlannerManager request context and
the V2 workflow through the shared planning/strategy boundary and fake physical
ports. They do not initialize CUDA, download assets, run a simulator, or prove
real GPU planning.

## Opt-in GPU smoke test

Provide a caller-owned schema-v2 request that contains both representative
world and attached geometry, plus an existing robot MotionGen config:

```bash
MANIPULATION_CUROBO_RUN_GPU=1 \
MANIPULATION_CUROBO_GPU_REQUEST=/absolute/path/request.yaml \
pytest -m gpu test/test_gpu_scene_bridge.py
```

The test performs no installation or download. See [`API.md`](API.md) for the
plugin parameters and [`MIGRATION.md`](MIGRATION.md) for incompatible removals.
