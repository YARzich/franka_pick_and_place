<!-- Responsibility: Explain the optional Isaac ROS dependency boundary and its ownership. -->
# manipulation_perception_isaac_ros

Dependency-only bundle for NVIDIA Isaac ROS components used alongside
`manipulation_perception`. It contains no nodes, detector logic, geometry,
configuration, assets or launch orchestration. The manifest makes the optional
GPU/NITROS dependency closure explicit and constrains its supported components
to the compatible Isaac ROS `4.1.x` line.

Robot and scenario packages decide whether this bundle is required. They own
component selection, topics, frames, remappings, model and engine paths, and
runtime asset validation. CPU-only consumers depend directly on
`manipulation_perception` and do not need this package.

Build only on a host with the NVIDIA Isaac ROS 4.1 repository installed:

```bash
colcon build --packages-select manipulation_perception_isaac_ros \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

TensorRT engines, checkpoints and model assets remain external and are checked
by the selected launch profile. Building this package does not download them.
