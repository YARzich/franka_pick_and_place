"""
Exercise the public MoveItConfigsBuilder loading convention for a nested profile.

Responsibility: Ensure franka_motion resources resolve without legacy package lookups.
"""

from config_test_utils import (
    ARM_JOINTS,
    PACKAGE_ROOT,
    load_package_yaml,
    load_yaml,
)
from moveit_configs_utils import MoveItConfigsBuilder


def test_setup_assistant_points_to_franka_motion_resources():
    """Setup Assistant metadata must not retain legacy package paths."""
    metadata = load_package_yaml('.setup_assistant')
    config = metadata['moveit_setup_assistant_config']

    assert config['URDF'] == {
        'package': 'franka_motion',
        'relative_path': 'description/urdf/panda_nomimic.urdf.xacro',
    }
    assert config['SRDF'] == {
        'relative_path': 'config/moveit/panda.srdf',
    }
    assert (PACKAGE_ROOT / config['URDF']['relative_path']).is_file()
    assert (PACKAGE_ROOT / config['SRDF']['relative_path']).is_file()


def test_builder_loads_nested_profile_through_public_file_arguments():
    """The builder must load each nested resource through supported public APIs."""
    config = (
        MoveItConfigsBuilder(
            'panda', package_name='franka_motion'
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics(
            file_path='config/moveit/kinematics.yaml'
        )
        .joint_limits(file_path='config/moveit/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
        )
        .trajectory_execution(
            file_path='config/moveit/moveit_controllers.yaml'
        )
        .planning_scene_monitor()
        .sensors_3d(file_path='config/moveit/sensors_3d_disabled.yaml')
        .to_moveit_configs()
        .to_dict()
    )

    assert '<robot name="panda">' in config['robot_description']
    assert '<group name="panda_arm">' in config['robot_description_semantic']
    assert config['planning_pipelines'] == ['ompl']
    assert config['default_planning_pipeline'] == 'ompl'
    assert config['sensors'] == ['disabled']
    assert config['disabled']['sensor_plugin'] == '~'
    assert set(config['robot_description_kinematics']) == {'panda_arm'}
    assert set(config['robot_description_planning']['joint_limits']) >= set(ARM_JOINTS)
    assert (
        config['robot_description_planning'][
            'shape_transform_cache_lookup_wait_time'
        ]
        == 0.5
    )
    assert config['moveit_simple_controller_manager']['panda_arm_controller'][
        'joints'
    ] == list(ARM_JOINTS)


def test_nested_ompl_profile_remains_explicit_and_arm_only():
    """The nested OMPL file is loaded by bringup as an explicit parameter map."""
    ompl = load_yaml('ompl_planning.yaml')

    assert ompl['planning_plugins'] == ['ompl_interface/OMPLPlanner']
    assert ompl['panda_arm']['default_planner_config'] == (
        'RRTConnectkConfigDefault'
    )


def test_builder_loads_only_the_explicit_optional_curobo_pipeline():
    """Ensure cuRobo uses the public builder convention without loading OMPL."""
    config = (
        MoveItConfigsBuilder('panda', package_name='franka_motion')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics(
            file_path='config/moveit/kinematics.yaml'
        )
        .joint_limits(file_path='config/moveit/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='curobo',
            pipelines=['curobo'],
        )
        .trajectory_execution(
            file_path='config/moveit/moveit_controllers.yaml'
        )
        .planning_scene_monitor()
        .sensors_3d(file_path='config/moveit/sensors_3d_disabled.yaml')
        .to_moveit_configs()
        .to_dict()
    )

    assert config['planning_pipelines'] == ['curobo']
    assert config['default_planning_pipeline'] == 'curobo'
    assert config['curobo']['planning_plugins'] == [
        'manipulation_curobo/CuRoboPlanner'
    ]
    assert 'ompl' not in config
