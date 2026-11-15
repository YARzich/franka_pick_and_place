"""
Launch the single-writer PlanningScene coordinator.

Responsibility:
    Declare coordinator configuration arguments and instantiate the sole
    PlanningScene mutation gateway for the selected MoveIt graph.

Does not own:
    Scene mutation policy, collision geometry, or workflow execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch the coordinator with one parameter file and consistent ROS time."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use ROS simulation time for node timestamps and diagnostics.',
    )
    config_file_argument = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('manipulation'),
                'config',
                'planning_scene_coordinator.yaml',
            ]
        ),
        description='PlanningScene coordinator parameter file.',
    )

    coordinator_node = Node(
        package='manipulation',
        executable='planning_scene_coordinator',
        name='planning_scene_coordinator',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    launch_description = LaunchDescription()
    launch_description.add_action(use_sim_time_argument)
    launch_description.add_action(config_file_argument)
    launch_description.add_action(coordinator_node)
    return launch_description
