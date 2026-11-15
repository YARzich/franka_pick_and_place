"""
Validate cross-file semantics of canonical MoveIt YAML resources.

Responsibility: Keep kinematics, limits, controllers, OMPL, and sensor profiles consistent.
"""

from config_test_utils import ARM_JOINTS, FINGER_JOINTS, load_urdf, load_yaml


def test_kinematics_contract_is_explicit_and_bounded():
    """KDL settings must remain explicit for the one IK-enabled group."""
    kinematics = load_yaml('kinematics.yaml')

    assert set(kinematics) == {'panda_arm'}
    arm = kinematics['panda_arm']
    assert arm['kinematics_solver'] == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    assert 0.0 < arm['kinematics_solver_search_resolution'] <= 0.01
    assert 0.0 < arm['kinematics_solver_timeout'] <= 0.1


def test_moveit_limits_are_complete_positive_and_conservative():
    """Overrides must cover all movable joints without exceeding URDF velocity."""
    config = load_yaml('joint_limits.yaml')
    assert 0.2 <= config['shape_transform_cache_lookup_wait_time'] <= 1.0
    configured = config['joint_limits']
    assert set(configured) == set(ARM_JOINTS + FINGER_JOINTS)

    urdf_limits = {
        joint.attrib['name']: float(joint.find('limit').attrib['velocity'])
        for joint in load_urdf().findall('joint')
        if joint.find('limit') is not None and 'velocity' in joint.find('limit').attrib
    }
    for name, limits in configured.items():
        assert limits['has_velocity_limits'] is True
        assert limits['has_acceleration_limits'] is True
        assert limits['has_jerk_limits'] is True
        assert 0.0 < limits['max_velocity'] <= urdf_limits[name]
        assert limits['max_acceleration'] > 0.0
        assert limits['max_jerk'] > 0.0


def test_controller_mapping_matches_arm_execution_contract():
    """The controller map must resolve the stable seven-joint action."""
    config = load_yaml('moveit_controllers.yaml')
    manager = config['moveit_simple_controller_manager']

    assert config['moveit_controller_manager'] == (
        'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    )
    assert manager['controller_names'] == ['panda_arm_controller']
    controller = manager['panda_arm_controller']
    assert controller == {
        'action_ns': 'follow_joint_trajectory',
        'type': 'FollowJointTrajectory',
        'default': True,
        'joints': list(ARM_JOINTS),
    }

    execution = config['trajectory_execution']
    assert execution['execution_duration_monitoring'] is True
    assert execution['allowed_execution_duration_scaling'] >= 1.0
    assert execution['allowed_goal_duration_margin'] >= 0.0
    assert execution['allowed_start_tolerance'] > 0.0


def test_ompl_exposes_only_the_executable_arm_group():
    """Load one known planner pipeline for the controller-owned arm group."""
    config = load_yaml('ompl_planning.yaml')

    assert config['planning_plugins'] == ['ompl_interface/OMPLPlanner']
    assert config['request_adapters'] == [
        'default_planning_request_adapters/ResolveConstraintFrames',
        'default_planning_request_adapters/ValidateWorkspaceBounds',
        'default_planning_request_adapters/CheckStartStateBounds',
        'default_planning_request_adapters/CheckStartStateCollision',
    ]
    assert config['response_adapters'] == [
        'default_planning_response_adapters/AddTimeOptimalParameterization',
        'default_planning_response_adapters/ValidateSolution',
        'default_planning_response_adapters/DisplayMotionPath',
    ]
    planner = config['planner_configs']['RRTConnectkConfigDefault']
    assert planner == {'type': 'geometric::RRTConnect', 'range': 0.0}
    assert config['panda_arm'] == {
        'default_planner_config': 'RRTConnectkConfigDefault',
        'planner_configs': ['RRTConnectkConfigDefault'],
        'longest_valid_segment_fraction': 0.001,
        'maximum_waypoint_distance': 0.01,
    }
    assert 'hand' not in config
    assert 'panda_arm_hand' not in config


def test_ompl_collision_checking_is_not_coarser_than_validate_solution():
    """The planner must see the same thin obstacles its validator rejects."""
    config = load_yaml('ompl_planning.yaml')
    arm = config['panda_arm']

    assert 'default_planning_response_adapters/ValidateSolution' in (
        config['response_adapters']
    )
    assert 0.0 < arm['longest_valid_segment_fraction'] <= 0.001
    # Keep each checked joint-space segment near the 0.015 m octomap voxel
    # resolution so a single voxel cannot fit between two checks.
    assert 0.0 < arm['maximum_waypoint_distance'] <= 0.015


def test_isaac_file_is_a_relaxed_node_scoped_override_only():
    """Isaac may relax timing, but may not redefine controllers or joints."""
    base = load_yaml('moveit_controllers.yaml')['trajectory_execution']
    override = load_yaml('isaac_trajectory_execution.yaml')

    assert set(override) == {'move_group'}
    parameters = override['move_group']['ros__parameters']
    assert set(parameters) == {'trajectory_execution'}
    execution = parameters['trajectory_execution']
    assert execution['execution_duration_monitoring'] is True
    assert execution['allowed_start_tolerance'] >= base['allowed_start_tolerance']
    assert execution['allowed_execution_duration_scaling'] >= (
        base['allowed_execution_duration_scaling']
    )
    assert execution['allowed_goal_duration_margin'] >= (
        base['allowed_goal_duration_margin']
    )


def test_sensor_free_profile_explicitly_skips_octomap_updaters():
    """Sensor-free leaves must disable the Jazzy updater without log filtering."""
    config = load_yaml('sensors_3d_disabled.yaml')

    assert config['octomap_resolution'] == 0.025
    assert config['octomap_frame'] == 'world'
    assert config['sensors'] == ['disabled']
    assert config['disabled'] == {'sensor_plugin': '~'}


def test_table_camera_profile_uses_the_tf_admitted_sensor_frame_cloud():
    """Octomap ray casting must consume the exact-stamp gated optical cloud."""
    config = load_yaml('sensors_3d_table_camera.yaml')

    assert config['octomap_resolution'] == 0.015
    assert config['octomap_frame'] == 'world'
    assert config['sensors'] == ['table_stereo_pointcloud']
    sensor = config['table_stereo_pointcloud']
    assert sensor['sensor_plugin'] == (
        'manipulation_moveit/FailClosedPointCloudOctomapUpdater'
    )
    assert sensor['point_cloud_topic'] == '/table_stereo/moveit_octomap_points'
    assert sensor['point_cloud_topic'] != '/table_stereo/points'
    assert sensor['max_range'] > 0.0
    assert sensor['point_subsample'] == 1
    assert sensor['padding_scale'] == 1.0
    assert sensor['padding_offset'] >= 2.0 * config['octomap_resolution']
    assert sensor['max_update_rate'] > 0.0


def test_servo_profile_uses_only_the_jazzy_ros_api_contract():
    """Keep Servo names, timing, output, smoothing, and safety explicit."""
    config = load_yaml('servo.yaml')

    assert config['move_group_name'] == 'panda_arm'
    assert config['pose_command_in_topic'] == '~/pose_target_cmds'
    assert config['status_topic'] == '~/status'
    assert config['joint_topic'] == '/joint_states'
    assert config['command_out_topic'] == (
        '/panda_arm_controller/joint_trajectory'
    )
    assert config['command_out_type'] == 'trajectory_msgs/JointTrajectory'
    assert config['publish_joint_positions'] is True
    assert config['publish_joint_velocities'] is True
    assert config['publish_joint_accelerations'] is False
    assert config['is_primary_planning_scene_monitor'] is False
    assert config['check_octomap_collisions'] is False
    assert config['check_collisions'] is True
    assert config['use_smoothing'] is True
    assert config['smoothing_filter_plugin_name'] == (
        'online_signal_smoothing::AccelerationLimitedPlugin'
    )
    assert 0.0 < config['publish_period'] < config['incoming_command_timeout']
    assert config['lower_singularity_threshold'] == 17.0
    assert config['lower_singularity_threshold'] < (
        config['hard_stop_singularity_threshold']
    )
