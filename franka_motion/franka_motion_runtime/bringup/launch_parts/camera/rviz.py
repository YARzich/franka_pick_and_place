"""
RViz launch action factory for table-camera visualization.

Responsibility:
- Create the delayed RViz action for table-camera perception scenarios.
- Pass use_sim_time, robot description and the camera RViz config into RViz.

Does not own:
- RViz config file contents.
- Perception, TF or MoveIt node construction.
"""

from __future__ import annotations

from dataclasses import dataclass

from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


@dataclass(frozen=True)
class RvizActions:
    """Conditional RViz node and its delayed start action."""

    node: Node
    delayed_start: TimerAction


def create_rviz_actions(
    configs: dict[str, LaunchConfiguration],
    moveit_config,
) -> RvizActions:
    """Create table-camera RViz with robot and MoveIt descriptions."""
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="franka_stereo_rviz",
        output="screen",
        parameters=[
            {"use_sim_time": configs["use_sim_time"]},
            moveit_config.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        arguments=["-d", configs["rviz_config"]],
        ros_arguments=["--log-level", "error", "--log-level", "tf2_buffer:=error"],
        condition=IfCondition(configs["start_rviz"]),
    )
    rviz_start_delayed = TimerAction(
        period=10.0,
        actions=[rviz_node],
    )
    return RvizActions(node=rviz_node, delayed_start=rviz_start_delayed)
