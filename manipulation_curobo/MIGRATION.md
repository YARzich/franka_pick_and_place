# Migration to 0.2

Version 0.2 intentionally removes every backend-specific task/action route.
The separate pose-planning action server, warm worker, direct execution-facing
runtime, static-world helpers, historical plugin alias and compatibility tests
are no longer installed. Select cuRobo only as a MoveIt planning pipeline behind
the common planning port.

Required profile changes:

1. Use only plugin id `manipulation_curobo/CuRoboPlanner`.
2. Remove `world_cfg` from MotionGen YAML. The complete immutable PlanningScene
   in each MoveIt request is authoritative.
3. For attached geometry, add `attached_link_map` and
   `attached_touch_links`, and reserve collision spheres on the mapped cuRobo
   proxy link in the robot config.
4. Do not duplicate MoveIt self-collision pairs in the backend profile. The
   bridge reads them from each request ACM and replaces the loaded robot
   config's ignore map with that exact set plus explicit proxy/touch pairs.
5. Treat unsupported scene features as an explicit backend capability error;
   do not fall back to a stale static world.
6. Set a non-empty `motion_gen.workspace_frame`; it must equal the immutable
   PlanningScene planning frame for pose and joint requests.

Clients that formerly called a backend-specific endpoint must instead submit
the existing high-level transfer intent. The application backend profile picks
the cuRobo MoveIt pipeline without changing task or execution contracts.

The plugin is now the only installed integration surface. Private helper
libraries and headers from earlier overlays are removed on install. Explicit
selection also performs an import/config/CUDA readiness probe during MoveIt
startup, before a task goal can be admitted.
