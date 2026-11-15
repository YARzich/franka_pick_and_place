"""
Launch the read-only manipulation system health monitor.

Responsibility:
    Declare the health profile and simulation-clock arguments and instantiate
    the monitor node with its package-owned default configuration.

Does not own:
    Health policy, diagnostics evaluation, or recovery actions.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the standalone diagnostics launch description."""
    package_share = get_package_share_directory('manipulation')
    default_config = os.path.join(package_share, 'config', 'system_health.yaml')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    config_arg = DeclareLaunchArgument('config', default_value=default_config)

    monitor_node = Node(
        package='manipulation',
        executable='system_health_monitor',
        name='system_health_monitor',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    launch_description = LaunchDescription()
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(config_arg)
    launch_description.add_action(monitor_node)
    return launch_description
