<!-- Responsibility: Record licensing boundaries for external Isaac ROS dependencies and assets. -->
# Third-Party Notice

This integration manifest is MIT-licensed. It does not vendor NVIDIA code,
containers, models or binaries. The directly referenced Isaac ROS 4.1 packages
declare Apache-2.0; their transitive CUDA, TensorRT, NITROS/GXF components and
model assets may have separate NVIDIA terms.

Upstream sources and license files remain authoritative:

- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
- https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation

FoundationPose model/data licenses are independent from the Apache-2.0 ROS
adapter. Review every deployed engine and checkpoint before redistribution.
