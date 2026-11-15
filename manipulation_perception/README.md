<!-- Responsibility: Explain package boundaries, integration requirements, and verification. -->
# manipulation_perception

Robot-independent perception building blocks for manipulation systems. The
package combines stamped stereo geometry, detector adapters, and 6D-pose
backend adapters while keeping them in separate Python namespaces:

- `manipulation_perception.common`: synchronization, geometry, stereo depth,
  point clouds, TF adapters, and diagnostics;
- `manipulation_perception.yolo`: optional Ultralytics/SAHI detector backend;
- `manipulation_perception.pose`: optional MegaPose/GigaPose worker adapters.
- `manipulation_perception.validation`: read-only runtime evidence contracts,
  provenance and pure correlation/diagnostic evaluation.

## Configuration boundary

The package intentionally contains no robot model, scenario topics, object
mesh, detector checkpoint, dataset identifier, or simulator configuration.
Every ROS topic/frame, robot workspace radius, object label, model path, and
external backend path is supplied by the scenario package. Nodes reject missing
required values before creating their runtime pipeline.

Related RGB, calibration, bbox, mask, depth, and pose messages are correlated
by exact `stamp + frame_id`; payload-only matching is not supported.

Local SGBM uses one exact-time left/right/CameraInfo sample. Its configured
left input and output optical frame must be identical; the right input frame is
validated independently. External disparity requires identical headers on the
outer disparity message, embedded image, and CameraInfo.

YOLO, MegaPose, GigaPose and FoundationPose integrations are optional runtime
backends with independent licenses. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the pinned source manifest
[dependencies.repos](dependencies.repos).

When the GigaPose coarse worker is configured as persistent, its pipeline
preloads the coarse model and MegaPose refiner sequentially at node startup.
The first inference waits for that bounded worker request sequence, so model
loading cannot overlap frame processing. Shutdown releases the wait and stops
both worker process groups. One-shot worker mode remains lazy and is never
prewarmed.

See [API.md](API.md) for executables and required parameters.

The optional GPU evidence probe is validation tooling, not a production
pipeline owner. A scenario supplies all graph names, frames, logical assets and
explicit runtime file paths. The resulting artifact proves one bounded stamped
runtime correlation and diagnostics window; it does not claim model accuracy or
provide replay data.
