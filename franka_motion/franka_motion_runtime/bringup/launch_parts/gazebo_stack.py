"""
Shared Gazebo robot-model and ros2_control launch composition.

Responsibility:
- Prepare one generated Panda model and its resolved integration paths.
- Group the simulator, bridge, state publisher, spawners and gripper actions.
- Delegate the established controller readiness chain without planner coupling.

Does not own:
- Gazebo bridge topic selection, simulator environment or world assets.
- MoveIt, cuRobo, PlanningScene or perception node construction.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_motion_runtime.bringup.launch_parts.control import (
    create_gazebo_controller_spawners,
    create_gazebo_gripper_driver,
    create_robot_state_publisher,
)
from franka_motion_runtime.bringup.launch_parts.descriptions import (
    generate_gazebo_robot_description,
    write_robot_urdf,
)
from franka_motion_runtime.bringup.launch_parts.events import (
    create_gazebo_control_events,
    create_shutdown_on_exit_event,
)
from franka_motion_runtime.bringup.launch_parts.paths import (
    GazeboLaunchPaths,
    create_gazebo_launch_paths,
)
from franka_motion_runtime.simulation.gazebo.actions import (
    create_gazebo_processes,
    create_spawn_robot_node,
)
from franka_motion_runtime.simulation.gazebo.scenarios import GazeboScenario


@dataclass(frozen=True)
class GazeboRobotModel:
    """Generated Panda description and paths for one immutable scenario."""

    paths: GazeboLaunchPaths
    description_xml: str

    @property
    def robot_description(self) -> dict[str, str]:
        """Return the ROS parameter overlay expected by robot consumers."""
        return {"robot_description": self.description_xml}


@dataclass(frozen=True)
class GazeboControlStack:
    """Named Gazebo/control actions shared by production and reference planners."""

    model: GazeboRobotModel
    headless_simulator: ExecuteProcess
    gui_simulator: ExecuteProcess
    bridge: ExecuteProcess
    robot_state_publisher: Node
    spawn_robot: Node
    joint_state_broadcaster_spawner: Node
    arm_controller_spawner: Node
    hand_controller_spawner: Node
    gripper_driver: Node
    simulator_exit_events: tuple[RegisterEventHandler, RegisterEventHandler]

    def create_motion_start_events(
        self,
        *,
        motion_backend_node: Node,
        motion_companion_nodes: Sequence[object] = (),
        planning_scene_node: Node | None = None,
    ) -> list[RegisterEventHandler]:
        """Release one motion backend after the unchanged controller chain."""
        return create_gazebo_control_events(
            spawn_robot_node=self.spawn_robot,
            joint_state_broadcaster_spawner=(
                self.joint_state_broadcaster_spawner
            ),
            panda_arm_spawner=self.arm_controller_spawner,
            panda_hand_spawner=self.hand_controller_spawner,
            gripper_driver=self.gripper_driver,
            move_group_node=motion_backend_node,
            motion_companion_nodes=motion_companion_nodes,
            planning_scene_node=planning_scene_node,
        )


def prepare_gazebo_robot_model(scenario: GazeboScenario) -> GazeboRobotModel:
    """Generate and persist the robot model required by one Gazebo leaf."""
    paths = create_gazebo_launch_paths(scenario)
    description_xml = generate_gazebo_robot_description(
        paths,
        scenario.base_pose,
    )
    write_robot_urdf(paths.tmp_urdf, description_xml)
    return GazeboRobotModel(paths=paths, description_xml=description_xml)


def create_gazebo_control_stack(
    model: GazeboRobotModel,
    *,
    use_sim_time: LaunchConfiguration,
    headless: LaunchConfiguration,
    bridge: ExecuteProcess,
    use_extended_timeouts: bool = False,
    robot_state_publisher_ros_arguments: list[str] | None = None,
) -> GazeboControlStack:
    """Create common Gazebo and ros2_control actions around a prepared model."""
    paths = model.paths
    headless_simulator, gui_simulator = create_gazebo_processes(
        world_path=paths.world,
        headless=headless,
    )
    robot_state_publisher = create_robot_state_publisher(
        use_sim_time=use_sim_time,
        robot_description=model.robot_description,
        ros_arguments=robot_state_publisher_ros_arguments,
    )
    spawn_robot = create_spawn_robot_node(paths.tmp_urdf)
    joint_state_spawner, arm_spawner, hand_spawner = (
        create_gazebo_controller_spawners(
            paths.controller_config,
            use_sim_time=use_sim_time,
            use_extended_timeouts=use_extended_timeouts,
        )
    )
    gripper_driver = create_gazebo_gripper_driver(
        use_sim_time=use_sim_time,
        gripper_config_path=paths.gripper_config,
    )
    simulator_exit_events = (
        create_shutdown_on_exit_event(
            target_action=headless_simulator,
            process_name="Gazebo headless server",
        ),
        create_shutdown_on_exit_event(
            target_action=gui_simulator,
            process_name="Gazebo GUI process",
        ),
    )
    return GazeboControlStack(
        model=model,
        headless_simulator=headless_simulator,
        gui_simulator=gui_simulator,
        bridge=bridge,
        robot_state_publisher=robot_state_publisher,
        spawn_robot=spawn_robot,
        joint_state_broadcaster_spawner=joint_state_spawner,
        arm_controller_spawner=arm_spawner,
        hand_controller_spawner=hand_spawner,
        gripper_driver=gripper_driver,
        simulator_exit_events=simulator_exit_events,
    )
