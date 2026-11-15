^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manipulation_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Isolate pose examples, inputs, prediction logs, derived meshes and caches in
  an instance-owned workspace; clean it only after workers stop and join.
* Preserve exact YOLO parameters until the corresponding UI control is edited.
* Bind stereo and pose TF buffers to the owning ROS clock for epoch reset.
* Count temporal detector confirmation once per track and processed frame.
* Clean up persistent worker descendants after the leader exits between requests,
  and prevent replacement if shutdown begins during cleanup.
* Retain Python worker/CLI leader identity until the last process-group signal,
  serialize repeated cleanup, and close signaling before reap. Capture one-shot
  stdout/stderr in temporary files to avoid implicit reap during concurrent stop.
* Reject non-finite or invalid-quaternion Detection3D poses before TF/publication,
  sharing the existing numeric validation with the MegaPose message builder.

0.1.0 (2026-07-14)
------------------
* Added reusable stamped stereo, detection, point-cloud, and 6D-pose
  components with backend-neutral ROS contracts.
* Added fail-closed finite and range validation for stereo, disparity, depth,
  table-plane, and point-cloud parameters.
* Added a scenario-driven, read-only GPU perception evidence probe with strict
  stamped correlation, diagnostic deltas, explicit provenance, and schemas.
* Preserved the source detection timestamp after TF conversion to the output
  pose frame.
