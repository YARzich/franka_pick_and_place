^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package franka_motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Detected observations now require a fresh, non-future, non-replayed source
  stamp within ``max_observation_age_sec`` (default/profile: 5 seconds), checked
  again after exact-stamp TF. ROS clock rollback clears the consumed-stamp
  barrier and waits for a new receipt; the configured provider is unchanged.
* Isaac launch selects an explicit ``FRANKA_ISAAC_PYTHON`` / ``isaac_python``;
  the runner discovers installed package resources in that interpreter and
  supports the ROS core bridge layout. Python 3.12 standard-library and venv
  paths survive ROS environment isolation; private user install defaults were
  removed from the runner and validation harness.
* Process wrappers observe child exit without reaping the group leader until
  descendant cleanup finishes, retire group signaling before reap, and ignore
  pinned zombies when checking bounded shutdown completion.
* Offline GigaPose dataset preparation now requires an explicit destination or
  ``FRANKA_GPU_GIGAPOSE_DATA_DIR``.
* Active-cancel validation now waits for fresh ``EXECUTING`` status from the
  configured FollowJointTrajectory controller while step zero is currently
  ``APPROACHING``. The accepted controller goal must be newer than the sequence
  submission baseline; terminal, stale, retained, or ambiguous status cannot
  trigger the probe. Logs include the controller goal UUID and accepted stamp.
  This proves a command accepted for execution, not measured nonzero velocity.
* Added required probe parameter ``probe.controller_action_name``, supplied by
  the Franka active-cancel launch leaf. A missing execution checkpoint preserves
  exact sequence-goal cancellation/containment and fails the runtime probe.
* Both MoveGroup factories replace the upstream ExecuteTrajectory capability
  with ``manipulation_moveit/CancelableExecuteTrajectoryCapability`` on the same
  canonical action. MoveIt 2.12.4 otherwise blocks cancellation until execution
  completes without stopping the controller.
* Aligned scene validation with V2 ownership: a canceled source object must remain
  in world geometry and must not be attached. Lease release does not remove a
  physical obstacle; coordinator quiescence checks remain mandatory.

0.2.0 (2026-07-23)
------------------
* Replaced the ``CuRoboPlanPose`` action, bridge, direct demos, and dedicated
  launch aliases with the optional ``curobo`` MoveIt PlannerManager pipeline.
  The same ``TransferObjectSequenceV2`` composition now selects OMPL or cuRobo
  without changing execution, gripper, scene, or lifecycle ownership.
* Made cuRobo a runtime-discovered optional package. Standard OMPL bringup no
  longer declares or loads ``manipulation_curobo``; explicit selection fails
  before process startup if the plugin is unavailable or the scenario contains
  the currently unsupported table-camera Octomap.
* Moved the cuRobo pipeline to ``config/moveit/curobo_planning.yaml`` and made
  the complete PlanningScene request the sole world source, including explicit
  Panda attached-object mapping; robot collision policy comes from the request
  ACM rather than a second YAML copy.
* Preserved raw detected stamped poses and absolute targets while adding
  catalog-declared local proper-rotation equivalence used only by
  ``TARGET_INITIAL_SOURCE``. The shipped cube profile declares all 24 proper
  cube rotations.
* Made resident MTC construct a workflow-aligned frame in its local scene copy,
  so grasp generation remains consistent for arbitrary detected orientation
  without rewriting the authoritative object pose.
* Removed the former ``franka_motion::cube_grasp_pose_policy`` C++ target and
  header; catalog symmetry and the local MTC frame replace its scenario-specific
  projection without changing raw observations.
* Replaced MoveIt's default ``MoveGroupMoveAction`` capability in every
  maintained Gazebo/Isaac leaf with the exact full-scene capability while
  retaining the canonical ``/move_action`` and OMPL/cuRobo pipeline selection.
* Removed the sequence client's configurable discovery, clock, goal-response,
  whole-result, and polling timeouts. These waits now continue until evidence or
  shutdown; only private bounded transport probes remain inside fail-stopped
  exact-goal cancellation/reconciliation.
* Added typed ``outcome=`` fields to production wait-exit logs.
* Removed retired V1/detected/cuRobo Python modules and bytecode from
  incremental installs instead of leaving importable compatibility paths.
* Guarded the MoveIt plan/execute split with fresh joint-state endpoint
  quiescence and post-plan scene start validation/replanning, and mapped the
  position-only Panda close command to neutral ``position=0.0`` and
  ``max_effort=0.0``.
* Applied and verified deterministic Isaac Franka position-drive gains and
  solver iterations for loaded USD and headless URDF assets, and tightened
  controller endpoint tolerances, preventing an underdamped articulation from
  reporting terminal trajectory success while joints are still moving. Arm
  and contact-safe finger tuning remain separate and retain authored effort
  limits.
* Added an object-scoped Isaac physical-attachment barrier: USD/PhysX mutations
  stay on the simulation thread, a pose-preserving fixed joint is acknowledged
  only after a physics step, PlanningScene mutation follows that ACK, and any
  ambiguous partial result quarantines the workflow until the complete Isaac
  scenario/stack is restarted. Restarting only the C++ workflow server node is
  not a safe way to clear that quarantine.

0.1.0 (2026-07-14)
------------------
* Established canonical Panda ownership for description, ros2_control,
  MoveIt configuration, Gazebo and Isaac bringup, task leaves, and runtime
  validation.
* Retained documented compatibility entrypoints while generic behavior moved
  to the manipulation packages.
* Added a fail-closed Gazebo action-backend process-death/restart profile with
  exact run-token, role, procfs identity and pidfd targeting.
* Hardened short-lived helper ownership against incomplete process-group setup
  and interrupted child reaping.
* Added a dedicated Gazebo GigaPose evidence profile and self-hosted workflow
  with exact stamped correlation, nested artifact validation, GPU identity and
  pre-run model digest binding.
