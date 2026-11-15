"""
Robot description generation and temporary URDF writing for launch scenarios.

Responsibility:
- Run the project xacro descriptions with scenario-specific mappings.
- Write generated robot descriptions to temporary URDF files consumed by launch actions.

Does not own:
- The xacro model structure or ros2_control plugin definitions.
- MoveIt configuration assembly or simulator process startup.
"""

from __future__ import annotations

from pathlib import Path
import subprocess

from franka_motion_runtime.bringup.launch_parts.paths import GazeboLaunchPaths, IsaacLaunchPaths


def generate_gazebo_robot_description(
    paths: GazeboLaunchPaths,
    base_pose: dict[str, str] | None = None,
) -> str:
    """Generate the Gazebo robot model at the selected REP-103 base pose."""
    command = [
        "xacro",
        str(paths.robot_xacro),
        f"initial_positions_file:={paths.initial_positions}",
        f"ros2_control_config:={paths.ros2_control_config}",
    ]
    if base_pose is not None:
        command.extend(
            [
                f"base_x:={base_pose['x']}",
                f"base_y:={base_pose['y']}",
                f"base_z:={base_pose['z']}",
                f"base_roll:={base_pose['roll']}",
                f"base_pitch:={base_pose['pitch']}",
                f"base_yaw:={base_pose['yaw']}",
            ]
        )
    return subprocess.check_output(command, text=True)


def generate_isaac_robot_description(
    paths: IsaacLaunchPaths,
    *,
    include_control: bool = True,
) -> str:
    """Generate the Isaac robot model with an optional ros2_control tag."""
    import xacro

    document = xacro.process_file(
        str(paths.panda_xacro),
        mappings={
            "initial_positions_file": str(paths.initial_positions),
            "ros2_control_plugin": "topic_based_ros2_control/TopicBasedSystem",
            "ros2_hand_control_type": "isaac",
            "joint_commands_topic": "/isaac_joint_commands",
            "joint_states_topic": "/isaac_joint_states",
            "generate_ros2_control_tag": str(include_control).lower(),
        },
    )
    return document.toxml()


def write_robot_urdf(tmp_urdf_path: Path, robot_description_xml: str) -> None:
    """Write one generated robot model consumed by a simulator or planner."""
    tmp_urdf_path.write_text(robot_description_xml, encoding="utf-8")
