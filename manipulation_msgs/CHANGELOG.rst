^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manipulation_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-07-23)
------------------
* Completed the breaking V2 migration by removing the V1 static-box,
  perception-specific and inner transfer actions/messages after repository
  consumers moved to ``TransferObjectSequenceV2``.
* Removed the backend-specific ``CuRoboPlanPose`` action; optional cuRobo
  planning now uses the common planning backend port and MoveIt planner plugin.
* Removed the no-longer-used ``sensor_msgs`` and ``trajectory_msgs``
  dependencies.
* Added an exact-current V2 wire snapshot and source/generated contract tests,
  while preserving the V1 snapshot byte-for-byte as installed historical
  evidence.
* Added exact incremental-install cleanup for generated artifacts of the six
  retired wire types.

0.2.0 (2026-07-22)
------------------
* Added the versioned ``TransferObjectStepV2`` and
  ``TransferObjectSequenceV2`` high-level intent contracts without changing or
  removing V1 wire definitions.
* Added typed V2 waiting feedback, migration mapping, repository consumer
  inventory, and explicit V1 removal gates.
* Preserved the V1 wire snapshot as historical evidence and added focused V2
  source/generated contract coverage.

0.1.0 (2026-07-14)
------------------
* Established the robot-independent task, scene-coordination, and health
  interface baseline.
* Added a reviewed semantic snapshot of every public ROS wire definition.
