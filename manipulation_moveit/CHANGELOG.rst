^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manipulation_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Retain exact-goal planning cancellation through request adapters and delayed
  context creation, joining its private watcher before terminal completion.
* Separate gripper physical effects from diagnostic labels and move numeric
  quiescence/clock policy into a private directly tested module.
* Reject negative and non-normalized cloud stamps before MessageFilter and
  continue processing later valid samples.
* Added a cancel-capable MoveGroup ``ExecuteTrajectory`` replacement with the
  existing action contract. A single owned worker serializes exact-goal
  dispatch, stop, and join while ROS cancellation callbacks stay responsive.
  Uncertain stop/dispatch failures retain the unresolved goal and permanently
  close admission instead of publishing false terminal-safe evidence.
* Preserve confirmed or unknown gripper evidence before cancellation, and keep
  scene ownership when physical release has not yet been followed by a
  confirmed detach.
* Reject zero/future joint-state timestamps and backwards ROS-clock epochs in
  physical quiescence checks. Replayed and stale samples cannot prove terminal
  safety; fresh samples can resume convergence within the accepted budget.

0.2.0 (2026-07-23)
------------------
* Removed the V1 ``TransferObject`` backend, public ports/configuration,
  ``transfer_backend`` export, legacy tests, and
  ``moveit_transfer_object_server`` executable after the repository consumer
  gate was cleared.
* Replaced the mixed ``moveit_motion_host`` with the Servo-only
  ``moveit_servo_task_host`` library and ``moveit_servo_task_server``
  executable; the dedicated process now requires one complete Servo profile.
* Split ``moveit_octomap_tf_gate`` into the narrow
  ``octomap_tf_gate_support`` target so Octomap safety no longer keeps a legacy
  task backend alive.
* Added a plan-only arm boundary with typed outcomes, bounded retry/backoff,
  scene-revision revalidation, complete immutable PlanningScene input, and a
  MoveGroup adapter that does not own physical execution.
* Added an exact-scene ``move_action`` capability that replaces the default
  MoveGroup move capability for V2, constructs a fresh full PlanningScene, and
  preserves normal OMPL/cuRobo planning-pipeline selection without stale world
  inheritance.
* Refresh robot state immediately before every plan and reject successful
  trajectories whose first waypoint does not match the authoritative snapshot.
* Hardened the split plan/execute handoff with fresh ``/joint_states`` endpoint
  quiescence and a post-plan full-scene start check that discards and replans a
  stale trajectory before physical dispatch.
* Extended the acceptance-relative arm budget through the terminal
  ``/joint_states`` convergence proof; a premature controller success can no
  longer wait forever and now leaves sticky fail-closed readiness on expiry.
* Added the direct C++ ``pick_lift_place`` V2 strategy with separate planning,
  trajectory, gripper, health, and coordinated-scene ports, exact lease gates,
  post-acceptance timeout evidence, typed recovery, and CPU-only fake tests.
* Added production V2 ExecuteTrajectory, GripperCommand, and typed-health ROS
  adapters with acceptance-relative physical budgets, exact-goal cancellation,
  generation-safe late-result ownership, and safe-stop/quarantine evidence.
* Encoded position-only gripper close as neutral ``position=0.0`` and
  ``max_effort=0.0`` while retaining allowed-stall and typed hold evidence.
* Linearized planning, physical-command, and scene-service dispatch against
  pause/cancel; long planner and accepted-command waits now service lease/log
  heartbeat without applying pause to an in-flight physical command.
* Added a stop-only lease-maintenance gate so renewal continues during pause,
  plus deterministic dispatch-race/heartbeat tests and incremental-install
  cleanup for removed V1 headers, libraries, configuration, and executable.
* Added executor-backed production scene-adapter tests proving unlimited
  discovery/response continuation and prompt cancel/shutdown without later
  scene requests or side effects.
* Closed the ready-endpoint/shutdown dispatch race, added waits beyond the
  removed five-second limits, froze MoveGroup transport at an unbounded
  deadline, and verified typed planning shutdown propagation.

0.1.0 (2026-07-14)
------------------
* Added the robot-independent MoveIt transfer backend and fail-closed
  point-cloud Octomap path.
