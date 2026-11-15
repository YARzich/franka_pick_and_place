"""
Shared Isaac launch-time settings and ros2_control composition.

Responsibility:
- Resolve the stable Isaac launch arguments once per OpaqueFunction context.
- Group state publication, ros2_control, controller spawners and clock gating.
- Delegate the established simulator/controller/motion readiness chain.

Does not own:
- Isaac process command construction or simulator assets.
- MoveIt, cuRobo, PlanningScene or perception node construction.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.control import (
    create_isaac_controller_spawners,
    create_isaac_gripper_driver,
    create_isaac_ros2_control_node,
    create_robot_state_publisher,
)
from franka_motion_runtime.bringup.launch_parts.events import create_isaac_start_events
from franka_motion_runtime.bringup.launch_parts.paths import IsaacLaunchPaths
from franka_motion_runtime.simulation.isaac.actions import (
    boolean_from_config,
    create_wait_for_clock_action,
)


@dataclass(frozen=True)
class IsaacRuntimeSettings:
    """Resolved scalar settings shared by one Isaac launch graph."""

    use_sim_time: bool
    headless: bool
    fix_base: bool
    wait_for_clock: bool
    time_steps: str
    isaac_python: str
    isaac_robot_prim: str
    isaac_experience: str
    rmw_implementation: str

    @classmethod
    def from_context(
        cls,
        context: LaunchContext,
        *,
        include_isaac_experience: bool,
    ) -> IsaacRuntimeSettings:
        """Resolve declared substitutions while preserving historical defaults."""

        def resolved(name: str) -> str:
            return LaunchConfiguration(name).perform(context)

        isaac_experience = (
            resolved("isaac_experience")
            if include_isaac_experience
            else ""
        )
        return cls(
            use_sim_time=boolean_from_config(resolved("use_sim_time")),
            headless=boolean_from_config(resolved("headless")),
            fix_base=boolean_from_config(resolved("fix_base")),
            wait_for_clock=boolean_from_config(resolved("wait_for_clock")),
            time_steps=resolved("time_steps"),
            isaac_python=resolved("isaac_python"),
            isaac_robot_prim=resolved("isaac_robot_prim"),
            isaac_experience=isaac_experience,
            rmw_implementation=resolved("rmw_implementation"),
        )


@dataclass(frozen=True)
class IsaacControlStack:
    """Named Isaac/control actions shared by production and reference planners."""

    settings: IsaacRuntimeSettings
    robot_state_publisher: Node
    ros2_control_node: Node
    joint_state_broadcaster_spawner: Node
    arm_controller_spawner: Node
    hand_controller_spawner: Node
    gripper_driver: Node
    wait_for_clock_action: ExecuteProcess | None

    def nodes_after_simulator(
        self,
        *additional_nodes: object,
    ) -> list[object]:
        """Return ROS actions in the historical startup order after Isaac begins."""
        control_gate = (
            self.wait_for_clock_action
            if self.settings.wait_for_clock
            else self.ros2_control_node
        )
        if control_gate is None:
            raise RuntimeError(
                "wait_for_clock is enabled but no clock gate action exists"
            )
        return [
            self.robot_state_publisher,
            *additional_nodes,
            control_gate,
        ]

    def create_motion_start_events(
        self,
        *,
        isaac_process: ExecuteProcess,
        motion_backend_node: Node,
        motion_companion_nodes: Sequence[object] = (),
        additional_nodes: Sequence[object] = (),
        planning_scene_node: Node | None = None,
    ) -> list[object]:
        """Release one motion backend after Isaac, clock and controllers."""
        ros_nodes_after_isaac = self.nodes_after_simulator(
            *additional_nodes
        )
        return create_isaac_start_events(
            isaac_process=isaac_process,
            ros_nodes_after_isaac=ros_nodes_after_isaac,
            wait_for_clock=self.wait_for_clock_action,
            wait_for_clock_enabled=self.settings.wait_for_clock,
            ros2_control_node=self.ros2_control_node,
            joint_state_broadcaster_spawner=(
                self.joint_state_broadcaster_spawner
            ),
            panda_arm_spawner=self.arm_controller_spawner,
            panda_hand_spawner=self.hand_controller_spawner,
            move_group_node=motion_backend_node,
            gripper_driver=self.gripper_driver,
            motion_companion_nodes=motion_companion_nodes,
            planning_scene_node=planning_scene_node,
        )


def create_isaac_control_stack(
    *,
    paths: IsaacLaunchPaths,
    settings: IsaacRuntimeSettings,
    robot_description: dict[str, str],
) -> IsaacControlStack:
    """Create common Isaac ROS actions around one robot description."""
    robot_state_publisher = create_robot_state_publisher(
        use_sim_time=settings.use_sim_time,
        robot_description=robot_description,
    )
    ros2_control_node = create_isaac_ros2_control_node(
        paths=paths,
        use_sim_time=settings.use_sim_time,
        robot_description=robot_description,
    )
    joint_state_spawner, arm_spawner, hand_spawner = (
        create_isaac_controller_spawners(
            paths,
            use_sim_time=settings.use_sim_time,
        )
    )
    gripper_driver = create_isaac_gripper_driver(
        paths=paths,
        use_sim_time=settings.use_sim_time,
    )
    wait_for_clock_action = create_wait_for_clock_action(
        settings.wait_for_clock
    )
    return IsaacControlStack(
        settings=settings,
        robot_state_publisher=robot_state_publisher,
        ros2_control_node=ros2_control_node,
        joint_state_broadcaster_spawner=joint_state_spawner,
        arm_controller_spawner=arm_spawner,
        hand_controller_spawner=hand_spawner,
        gripper_driver=gripper_driver,
        wait_for_clock_action=wait_for_clock_action,
    )
