^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package manipulation_curobo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Reject unknown configured collision-checker names during backend readiness.
* Retain worker process-group ownership after the leader exits so timeout,
  cancellation, normal completion and owner shutdown also stop helper children.
* Cover real signal-resistant descendants and preserve the leader's exit code.

0.2.0 (2026-07-23)
------------------
* Made the MoveIt PlannerManager the sole integration path.
* Added request-owned full PlanningScene serialization, scene-aware MotionGen
  world updates, attached geometry and typed failure mapping.
* Removed the backend-specific action server, static-world runtime and plugin
  compatibility alias.
* Added a side-effect-free startup readiness probe for the selected Python,
  imports, config and CUDA device before request admission.
* Required one workspace/planning frame for pose and joint requests and added
  full-start-state, first-point, bounds and PlanningScene response validation.
* Removed helper headers/libraries from the installed CMake surface.
* Declared and linked the direct Eigen build dependency used by private scene
  serialization without exporting a second CMake integration surface.
* Cleaned historical 0.1.0 Python package metadata during incremental installs.
* Kept MotionGen initialization/warmup outside the request planning budget
  while retaining interruptible cancellation and shutdown.
* Added CPU integration coverage for the real PlannerManager context, accepted
  cancellation, and the V2 workflow through common fake physical ports.

0.1.0 (2026-07-14)
------------------
* Added the optional cuRobo planning reference backend and MoveIt planner
  plugin without production-workflow ownership.
