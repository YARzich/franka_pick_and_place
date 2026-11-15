"""
Gazebo table-camera entity and bridge factories.

Responsibility:
- Own the Franka scenario's camera pose and stable CameraInfo bridge contract.
- Create the camera entity-spawn action and dedicated bridge process action.

Does not own:
- Perception consumers or simulator/controller/MoveIt startup ordering.
"""

from __future__ import annotations

from pathlib import Path

from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import yaml

CAMERA_NAME = "table_camera"
CAMERA_POSE = {
    "x": "1.0",
    "y": "0.0",
    "z": "1.35",
    "roll": "0.0",
    "pitch": "0.7853981634",
    "yaw": "3.14159",
}
LEFT_CAMERA_INFO_TOPIC = "/table_stereo/left/camera_info"
RIGHT_CAMERA_INFO_TOPIC = "/table_stereo/right/camera_info"


def write_table_camera_bridge_config(
    tmp_bridge_config_path: Path,
    gz_clock_topic: str,
) -> None:
    """Write the exact clock and stereo CameraInfo bridge contract."""
    bridge_config = [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": gz_clock_topic,
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
            "lazy": False,
            "qos_profile": "CLOCK",
        },
        {
            "ros_topic_name": LEFT_CAMERA_INFO_TOPIC,
            "gz_topic_name": LEFT_CAMERA_INFO_TOPIC,
            "ros_type_name": "sensor_msgs/msg/CameraInfo",
            "gz_type_name": "gz.msgs.CameraInfo",
            "direction": "GZ_TO_ROS",
            "lazy": False,
        },
        {
            "ros_topic_name": RIGHT_CAMERA_INFO_TOPIC,
            "gz_topic_name": RIGHT_CAMERA_INFO_TOPIC,
            "ros_type_name": "sensor_msgs/msg/CameraInfo",
            "gz_type_name": "gz.msgs.CameraInfo",
            "direction": "GZ_TO_ROS",
            "lazy": False,
        },
    ]
    tmp_bridge_config_path.write_text(
        yaml.safe_dump(bridge_config, sort_keys=False),
        encoding="utf-8",
    )


def create_spawn_camera_node(camera_sdf_path: Path) -> Node:
    """Create the ``ros_gz_sim`` action that spawns the fixed table camera."""
    return Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            str(camera_sdf_path),
            "-name",
            CAMERA_NAME,
            "-allow_renaming",
            "false",
            "-x",
            CAMERA_POSE["x"],
            "-y",
            CAMERA_POSE["y"],
            "-z",
            CAMERA_POSE["z"],
            "-R",
            CAMERA_POSE["roll"],
            "-P",
            CAMERA_POSE["pitch"],
            "-Y",
            CAMERA_POSE["yaw"],
        ],
        output="screen",
    )


def create_table_camera_bridge_node(
    bridge_wrapper_path: Path,
    tmp_bridge_config_path: Path,
) -> ExecuteProcess:
    """Create the managed bridge process for clock and CameraInfo topics."""
    return ExecuteProcess(
        cmd=[
            str(bridge_wrapper_path),
            "--bridge-name",
            "clock_camera_bridge",
            "--config-file",
            str(tmp_bridge_config_path),
            "--log-level",
            "warn",
        ],
        output="log",
    )
