"""
ros2_control, controller spawner and gripper launch node factories.

Responsibility:
- Create robot_state_publisher, ros2_control and controller spawner actions.
- Apply the selected simulation clock policy to every created ROS node.
- Create simulator-specific gripper driver nodes with the correct parameter files.

Does not own:
- Controller YAML contents or hardware plugin implementation.
- Startup ordering between controllers and planning/perception nodes.
"""

from __future__ import annotations

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from franka_motion_runtime.bringup.launch_parts.constants import (
    CONTROLLER_MANAGER,
    CONTROL_PACKAGE,
    GAZEBO_GRIPPER_COMMAND_LIMITS,
    JOINT_STATE_BROADCASTER,
    PANDA_ARM_CONTROLLER,
    PANDA_HAND_CONTROLLER,
)
from franka_motion_runtime.bringup.launch_parts.paths import IsaacLaunchPaths


def create_robot_state_publisher(
    *,
    use_sim_time: LaunchConfiguration | bool,
    robot_description: dict[str, str],
    ros_arguments: list[str] | None = None,
) -> Node:
    """Create robot_state_publisher with a controller-rate TF ceiling."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        ros_arguments=ros_arguments,
        parameters=[
            {"use_sim_time": use_sim_time, "publish_frequency": 100.0},
            robot_description,
        ],
    )


def create_gazebo_controller_spawner(
    controller_name: str,
    *extra_arguments: str,
    use_sim_time: LaunchConfiguration | bool = True,
    controller_manager_timeout: str | None = None,
    switch_timeout: str | None = None,
    service_call_timeout: str | None = None,
) -> Node:
    """Create one Gazebo controller spawner with optional bounded timeouts."""
    arguments = [
        controller_name,
        "--controller-manager",
        CONTROLLER_MANAGER,
        "--activate",
        *extra_arguments,
    ]
    if controller_manager_timeout is not None:
        arguments.extend(["--controller-manager-timeout", controller_manager_timeout])
    if switch_timeout is not None:
        arguments.extend(["--switch-timeout", switch_timeout])
    if service_call_timeout is not None:
        arguments.extend(["--service-call-timeout", service_call_timeout])

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )


def create_gazebo_controller_spawners(
    controller_config_path,
    *,
    use_sim_time: LaunchConfiguration | bool = True,
    use_extended_timeouts: bool = False,
) -> tuple[Node, Node, Node]:
    """Create the ordered broadcaster, arm and hand Gazebo spawners."""
    timeout_kwargs = {}
    if use_extended_timeouts:
        timeout_kwargs = {
            "controller_manager_timeout": "120.0",
            "switch_timeout": "30.0",
            "service_call_timeout": "30.0",
        }
    joint_state_broadcaster_spawner = create_gazebo_controller_spawner(
        JOINT_STATE_BROADCASTER,
        use_sim_time=use_sim_time,
        **timeout_kwargs,
    )
    panda_arm_spawner = create_gazebo_controller_spawner(
        PANDA_ARM_CONTROLLER,
        "--param-file",
        str(controller_config_path),
        use_sim_time=use_sim_time,
        **timeout_kwargs,
    )
    panda_hand_spawner = create_gazebo_controller_spawner(
        PANDA_HAND_CONTROLLER,
        "--param-file",
        str(controller_config_path),
        use_sim_time=use_sim_time,
        **timeout_kwargs,
    )
    return joint_state_broadcaster_spawner, panda_arm_spawner, panda_hand_spawner


def create_isaac_ros2_control_node(
    *,
    paths: IsaacLaunchPaths,
    use_sim_time: bool,
    robot_description: dict[str, str],
) -> Node:
    """Create the managed topic-based ros2_control node for Isaac Sim."""
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(str(paths.ros2_control_config), allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        prefix=f"{paths.managed_process_wrapper} --child-name controller_manager --shutdown-signal kill --",
    )


def create_isaac_controller_spawner(
    *,
    paths: IsaacLaunchPaths,
    controller_name: str,
    use_sim_time: bool = True,
    use_param_file: bool = False,
) -> Node:
    """Create one managed Isaac controller spawner."""
    arguments = [
        controller_name,
        "--controller-manager",
        CONTROLLER_MANAGER,
        "--activate",
    ]
    if use_param_file:
        arguments.extend(["--param-file", str(paths.ros2_control_config)])
    arguments.extend(["--controller-manager-timeout", "120"])

    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        prefix=f"{paths.managed_process_wrapper} --child-name controller_spawner --shutdown-signal kill --",
    )


def create_isaac_controller_spawners(
    paths: IsaacLaunchPaths,
    *,
    use_sim_time: bool = True,
) -> tuple[Node, Node, Node]:
    """Create the ordered broadcaster, arm and hand Isaac spawners."""
    joint_state_broadcaster_spawner = create_isaac_controller_spawner(
        paths=paths,
        controller_name=JOINT_STATE_BROADCASTER,
        use_sim_time=use_sim_time,
    )
    panda_arm_spawner = create_isaac_controller_spawner(
        paths=paths,
        controller_name=PANDA_ARM_CONTROLLER,
        use_sim_time=use_sim_time,
        use_param_file=True,
    )
    panda_hand_spawner = create_isaac_controller_spawner(
        paths=paths,
        controller_name=PANDA_HAND_CONTROLLER,
        use_sim_time=use_sim_time,
        use_param_file=True,
    )
    return joint_state_broadcaster_spawner, panda_arm_spawner, panda_hand_spawner


def create_gazebo_gripper_driver(
    *,
    use_sim_time: LaunchConfiguration,
    gripper_config_path,
) -> Node:
    """Create the canonical gripper action adapter for Gazebo limits."""
    return Node(
        package=CONTROL_PACKAGE,
        executable="panda_gripper_driver",
        name="panda_gripper_driver",
        output="screen",
        parameters=[
            str(gripper_config_path),
            {
                "use_sim_time": use_sim_time,
                **GAZEBO_GRIPPER_COMMAND_LIMITS,
            },
        ],
    )


def create_isaac_gripper_driver(
    *,
    paths: IsaacLaunchPaths,
    use_sim_time: bool,
) -> Node:
    """Create the canonical gripper action adapter for Isaac feedback."""
    return Node(
        package=CONTROL_PACKAGE,
        executable="panda_gripper_driver",
        name="panda_gripper_driver",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            str(paths.gripper_config),
        ],
    )
