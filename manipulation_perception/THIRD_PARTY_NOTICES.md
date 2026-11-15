<!-- Responsibility: Record license boundaries of optional perception runtimes and assets. -->
# Third-Party Notices

The source authored in this ROS package is MIT-licensed. Optional inference
runtimes, models, datasets and generated engine files retain their own terms;
this package neither vendors nor relicenses them.

## Detector Backends

The YOLO adapter lazily imports the external `ultralytics` package. Its
open-source distribution is AGPL-3.0 and Ultralytics also offers separate
commercial terms. AGPL-3.0 is not a permissive MIT/BSD/Apache license and can
impose source-disclosure obligations on a deployed application. SAHI and any
selected checkpoint have their own license boundaries. Review the exact
installed versions and deployment model before enabling this backend.

## Pose Backends

- MegaPose at the pinned `f3b8e1247f133f3d098833a251b8f2d744c03e1f`
  revision declares [Apache-2.0][megapose-license] for its code.
- GigaPose at the pinned `17fcf97f493f79e56a215ab10ebff16d95cfe34b`
  revision declares [MIT][gigapose-license] for its code.
- DINOv2 at the pinned `ebc1cba1096de0a5388527817cf1e5d4d3896699`
  revision uses [CC-BY-NC-4.0][dinov2-pinned-license]. It is a non-commercial
  dependency of the documented GigaPose environment, not an Apache-2.0
  component. Later upstream DINOv2 revisions, including
  `7764ea0f912e53c92e82eb78a2a1631e92725fc8`, have an
  [Apache-2.0 license][dinov2-later-license]; migrating the pinned environment
  to that alternative requires a separate compatibility check. This
  publication preparation does not change the dependency pin.
- NVIDIA Isaac ROS pose-estimation packages are Apache-2.0.
- The separate NVlabs FoundationPose research repository uses the NVIDIA
  Source Code License; models and NGC assets can have additional terms.

The pinned source revisions used when preparing external environments are in
`dependencies.repos`. Datasets, checkpoints, meshes and transitive Python/CUDA
dependencies must be reviewed independently.

The bundled scenario-specific YOLO checkpoints are outside this package in
`franka_motion/models`. They explicitly record AGPL-3.0 and have separate
[model notices](../franka_motion/models/THIRD_PARTY_NOTICES.md).

## Replacement Boundary

These packages are optional implementations behind stamped detector and pose
ROS contracts. A deployment that cannot accept an external license should use
another backend with the same messages, topics, exact stamp/frame semantics
and diagnostics contract.

Upstream references:

- https://github.com/ultralytics/ultralytics/blob/main/LICENSE
- https://github.com/obss/sahi
- https://github.com/megapose6d/megapose6d
- https://github.com/nv-nguyen/gigaPose
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation
- https://github.com/NVlabs/FoundationPose

[megapose-license]: https://github.com/megapose6d/megapose6d/blob/f3b8e1247f133f3d098833a251b8f2d744c03e1f/LICENSE
[gigapose-license]: https://github.com/nv-nguyen/gigaPose/blob/17fcf97f493f79e56a215ab10ebff16d95cfe34b/LICENSE
[dinov2-pinned-license]: https://github.com/facebookresearch/dinov2/blob/ebc1cba1096de0a5388527817cf1e5d4d3896699/LICENSE
[dinov2-later-license]: https://github.com/facebookresearch/dinov2/blob/7764ea0f912e53c92e82eb78a2a1631e92725fc8/LICENSE
