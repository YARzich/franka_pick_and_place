"""
Test compatibility launch helpers and their public argument surface.

Responsibility: Verify canonical resource ownership, stable arguments, wiring, and documentation.
"""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch.utilities import perform_substitutions
import yaml

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(0, str(_PACKAGE_ROOT / 'launch'))
sys.path.insert(0, str(_PACKAGE_ROOT / 'test' / 'launch'))

from franka_launch_test_utils import DEFAULT_RUNTIME_TIMEOUT  # noqa: E402
from franka_motion_parts.camera.arguments import (  # noqa: E402
    create_gazebo_table_camera_launch_arguments,
    create_isaac_table_camera_launch_arguments,
)
from franka_motion_parts.camera.image_bridge import (  # noqa: E402
    ImageBridgeActions,
    image_bridge_shutdown_prefix,
)
from franka_motion_parts.camera.modes import create_pipeline_conditions  # noqa: E402
from franka_motion_parts.camera.paths import create_table_camera_paths  # noqa: E402
from franka_motion_parts.camera.perception import PerceptionActions  # noqa: E402
from franka_motion_parts.camera.pipeline import create_camera_pipeline_actions  # noqa: E402
from franka_motion_parts.camera.pose_estimators import (  # noqa: E402
    FoundationPoseActions,
    MegaPoseActions,
    PoseEstimatorInputActions,
)
from franka_motion_parts.camera.rviz import RvizActions  # noqa: E402
from franka_motion_parts.camera.stereo_depth import EssActions  # noqa: E402
from franka_motion_parts.camera.tf import CameraTfActions  # noqa: E402
from franka_motion_parts.camera.validation import create_table_camera_validation_actions  # noqa: E402
from franka_motion_parts.camera.yolo import yolo_shutdown_prefix  # noqa: E402
from franka_motion_parts.constants import (  # noqa: E402
    PERCEPTION_PACKAGE,
    PERCEPTION_POSE_PACKAGE,
    PERCEPTION_YOLO_PACKAGE,
)
from franka_motion_parts.gazebo.scenarios import GazeboScenario  # noqa: E402
import franka_motion_parts.paths as launch_paths  # noqa: E402
import franka_motion_parts.camera.pose_estimators as pose_estimators  # noqa: E402
import franka_motion_parts.camera.stereo_depth as stereo_depth  # noqa: E402
import franka_motion_parts.camera.yolo as yolo  # noqa: E402


def _environment_variable_name(argument: DeclareLaunchArgument) -> str | None:
    """Return the sole EnvironmentVariable substitution's name, if that is the default."""
    default = argument.default_value
    if default is None or len(default) != 1:
        return None
    substitution = default[0]
    if not isinstance(substitution, EnvironmentVariable):
        return None
    return _text_substitutions_to_string(substitution.name)


def _default_value(argument: DeclareLaunchArgument) -> str | None:
    """
    Resolve a literal default, treating an environment-backed one as None.

    Performing an EnvironmentVariable substitution here would make this check
    depend on whatever happens to be exported in the test-runner's shell.
    """
    if argument.default_value is None or _environment_variable_name(argument) is not None:
        return None
    return perform_substitutions(LaunchContext(), argument.default_value)


def _text_substitutions_to_string(substitutions) -> str:
    return ''.join(
        substitution.text
        for substitution in substitutions
        if isinstance(substitution, TextSubstitution)
    )


def _launch_configuration_name(config: LaunchConfiguration) -> str:
    return _text_substitutions_to_string(config.variable_name)


def _argument_defaults(arguments: list[DeclareLaunchArgument]) -> dict[str, str]:
    return {argument.name: _default_value(argument) for argument in arguments}


def _argument_names(arguments: list[DeclareLaunchArgument]) -> list[str]:
    return [argument.name for argument in arguments]


def _condition_value(condition, mode: str) -> bool:
    context = LaunchContext()
    context.launch_configurations['mode'] = mode
    return condition.evaluate(context)


def _canonical_camera_paths(
    *,
    bringup_share: str = '/share/franka_motion',
):
    return create_table_camera_paths(Path(bringup_share))


def test_runtime_paths_use_self_contained_scenario_package(monkeypatch):
    """All robot, simulator and MoveIt resources must resolve locally."""
    monkeypatch.setattr(
        launch_paths,
        'get_package_share_directory',
        lambda package_name: f'/share/{package_name}',
    )
    monkeypatch.setattr(
        launch_paths,
        'get_package_prefix',
        lambda package_name: f'/prefix/{package_name}',
    )

    gazebo = launch_paths.create_gazebo_launch_paths(
        GazeboScenario(
            world_file='table_world.sdf',
            tmp_urdf_path=Path('/tmp/panda.urdf'),
            tmp_bridge_config_path=Path('/tmp/bridge.yaml'),
        )
    )
    isaac = launch_paths.create_isaac_launch_paths()

    assert gazebo.semantic_config == Path(
        '/share/franka_motion/config/moveit/panda.srdf'
    )
    assert gazebo.kinematics_config == Path(
        '/share/franka_motion/config/moveit/kinematics.yaml'
    )
    assert gazebo.ros2_control_config == Path(
        '/share/franka_motion/config/control/panda_ros2_control_gazebo.yaml'
    )
    assert gazebo.gripper_config == Path(
        '/share/franka_motion/config/control/panda_gripper_driver.yaml'
    )
    assert gazebo.package_share == Path('/share/franka_motion')
    assert gazebo.robot_xacro == Path(
        '/share/franka_motion/description/urdf/panda_gazebo.urdf.xacro'
    )
    assert gazebo.managed_process_wrapper == Path(
        '/prefix/franka_motion/lib/franka_motion/'
        'managed_process_wrapper.py'
    )
    assert isaac.initial_positions == Path(
        '/share/franka_motion/config/control/initial_positions.yaml'
    )
    assert isaac.moveit_trajectory_execution_config == Path(
        '/share/franka_motion/config/moveit/isaac_trajectory_execution.yaml'
    )
    assert isaac.panda_xacro == Path(
        '/share/franka_motion/description/urdf/panda_isaac.urdf.xacro'
    )
    assert gazebo.world == Path(
        '/share/franka_motion/worlds/gazebo/table_world.sdf'
    )
    assert gazebo.simulator_share == Path('/share/franka_motion')
    assert isaac.panda_usd == Path(
        '/share/franka_motion/worlds/isaac/FrankaPanda/franka.usd'
    )
    assert isaac.isaac_script == Path(
        '/share/franka_motion/scripts/run_franka_isaac_sim.py'
    )
    assert isaac.isaac_worlds == Path('/share/franka_motion/worlds/isaac')


def test_table_camera_launch_arguments_keep_public_overrides(monkeypatch):
    paths = _canonical_camera_paths()

    gazebo_configs, gazebo_arguments = create_gazebo_table_camera_launch_arguments(paths)
    gazebo_defaults = _argument_defaults(gazebo_arguments)

    assert _launch_configuration_name(gazebo_configs['mode']) == 'mode'
    assert gazebo_defaults['mode'] == 'gigaPose'
    assert gazebo_defaults['table_camera_params_file'] == str(paths.perception_config)
    assert gazebo_defaults['point_cloud_stride'] == '4'
    assert gazebo_defaults['run_every_n_frames'] == '1'
    assert gazebo_defaults['fastdds_builtin_transports'] == 'DEFAULT'
    assert gazebo_defaults['gigapose_data_dir'] is None
    assert gazebo_defaults['gigapose_checkpoint_path'] == ''
    assert gazebo_defaults['gigapose_min_matching_score'] == '0.10'

    isaac_configs, isaac_arguments = create_isaac_table_camera_launch_arguments(paths)
    isaac_defaults = _argument_defaults(isaac_arguments)

    assert _launch_configuration_name(isaac_configs['mode']) == 'mode'
    assert isaac_defaults['mode'] == 'gigaPose'
    assert isaac_defaults['table_camera_params_file'] == str(paths.perception_config)
    assert 'fastdds_builtin_transports' not in isaac_defaults
    assert isaac_defaults['gigapose_data_dir'] == gazebo_defaults['gigapose_data_dir']
    assert isaac_defaults['gigapose_checkpoint_path'] == ''
    assert isaac_defaults['gigapose_min_matching_score'] == '0.10'


_ENVIRONMENT_VARIABLE_BY_BACKEND_PATH_ARGUMENT = {
    'yolo_python': 'FRANKA_GPU_YOLO_PYTHON',
    'megapose_python': 'FRANKA_GPU_MEGAPOSE_PYTHON',
    'megapose_data_dir': 'FRANKA_GPU_MEGAPOSE_DATA_DIR',
    'gigapose_python': 'FRANKA_GPU_GIGAPOSE_PYTHON',
    'gigapose_repo_dir': 'FRANKA_GPU_GIGAPOSE_REPO_DIR',
    'gigapose_dinov2_repo_dir': 'FRANKA_GPU_DINOV2_REPO_DIR',
    'gigapose_data_dir': 'FRANKA_GPU_GIGAPOSE_DATA_DIR',
    'ess_engine_file_path': 'FRANKA_GPU_ESS_ENGINE',
    'foundationpose_refine_model_file_path': 'FRANKA_GPU_FOUNDATIONPOSE_REFINE_MODEL',
    'foundationpose_refine_engine_file_path': 'FRANKA_GPU_FOUNDATIONPOSE_REFINE_ENGINE',
    'foundationpose_score_model_file_path': 'FRANKA_GPU_FOUNDATIONPOSE_SCORE_MODEL',
    'foundationpose_score_engine_file_path': 'FRANKA_GPU_FOUNDATIONPOSE_SCORE_ENGINE',
}


def test_backend_path_arguments_are_required_and_machine_neutral():
    """External backend paths must resolve from a named variable, not a literal path."""
    paths = _canonical_camera_paths()

    for create_arguments in (
        create_gazebo_table_camera_launch_arguments,
        create_isaac_table_camera_launch_arguments,
    ):
        _, arguments = create_arguments(paths)
        by_name = {argument.name: argument for argument in arguments}
        environment_backed_names = {
            argument.name
            for argument in arguments
            if _environment_variable_name(argument) is not None
        }
        assert environment_backed_names == set(_ENVIRONMENT_VARIABLE_BY_BACKEND_PATH_ARGUMENT)

        for argument_name, expected_variable in (
            _ENVIRONMENT_VARIABLE_BY_BACKEND_PATH_ARGUMENT.items()
        ):
            argument = by_name[argument_name]
            assert _environment_variable_name(argument) == expected_variable
            assert expected_variable in argument.description
            assert 'models/README.md' in argument.description
            # The EnvironmentVariable substitution must carry no fallback of
            # its own: an unset variable must still fail closed instead of
            # silently resolving to a path baked into the source tree.
            assert argument.default_value[0].default_value is None

        for argument in arguments:
            if argument.name in _ENVIRONMENT_VARIABLE_BY_BACKEND_PATH_ARGUMENT:
                continue
            assert '/home/' not in _default_value(argument)


def test_table_camera_launch_arguments_keep_public_argument_order(monkeypatch):
    paths = _canonical_camera_paths()
    gazebo_configs, gazebo_arguments = create_gazebo_table_camera_launch_arguments(paths)
    isaac_configs, isaac_arguments = create_isaac_table_camera_launch_arguments(paths)

    gazebo_names = _argument_names(gazebo_arguments)
    isaac_names = _argument_names(isaac_arguments)
    gazebo_only_names = ['fastdds_builtin_transports']
    expected_isaac_names = [
        'use_sim_time',
        'headless',
        'start_rviz',
        'table_camera_params_file',
        'rviz_config',
        'mode',
        'point_cloud_topic',
        'point_cloud_frame',
        'point_cloud_stride',
        'run_every_n_frames',
        'workspace_frame',
        'robot_base_frame',
        'robot_reach_m',
        'workspace_extra_ratio',
        'detection_bbox_topic',
        'detection_bbox_stamped_topic',
        'detection_bbox_timeout_sec',
        'yolo_python',
        'yolo_model_path',
        'yolo_mask_model_path',
        'yolo_confidence',
        'yolo_iou',
        'yolo_max_det',
        'yolo_device',
        'yolo_use_world',
        'yolo_open_vocab_classes',
        'yolo_imgsz',
        'yolo_run_every_n_frames',
        'yolo_agnostic_nms',
        'yolo_augment',
        'yolo_class_whitelist',
        'yolo_sahi_slice_width',
        'yolo_sahi_slice_height',
        'yolo_sahi_overlap_ratio',
        'yolo_min_box_area_ratio',
        'yolo_max_box_area_ratio',
        'yolo_temporal_confirm_frames',
        'yolo_temporal_iou_threshold',
        'megapose_python',
        'megapose_data_dir',
        'megapose_model',
        'megapose_mesh_file_path',
        'megapose_mesh_units',
        'megapose_object_label',
        'megapose_depth_mode',
        'megapose_bbox_timeout_sec',
        'megapose_run_every_n_frames',
        'megapose_min_inference_interval_sec',
        'megapose_process_timeout_sec',
        'megapose_persistent_worker',
        'megapose_bsz_images',
        'megapose_bsz_objects',
        'gigapose_python',
        'gigapose_repo_dir',
        'gigapose_dinov2_repo_dir',
        'gigapose_data_dir',
        'gigapose_checkpoint_path',
        'gigapose_dataset_name',
        'gigapose_object_id',
        'gigapose_template_count',
        'gigapose_max_num_dets_per_forward',
        'gigapose_translation_scale',
        'gigapose_num_hypotheses',
        'gigapose_min_matching_score',
        'gigapose_persistent_worker',
        'gigapose_refiner_bsz_images',
        'gigapose_refiner_bsz_objects',
        'gigapose_refiner_iterations',
        'ess_engine_file_path',
        'ess_threshold',
        'foundationpose_refine_model_file_path',
        'foundationpose_refine_engine_file_path',
        'foundationpose_score_model_file_path',
        'foundationpose_score_engine_file_path',
        'foundationpose_mesh_file_path',
        'foundationpose_texture_path',
    ]
    expected_gazebo_names = [
        *expected_isaac_names[:5],
        *gazebo_only_names,
        *expected_isaac_names[5:],
    ]

    assert isaac_names == expected_isaac_names
    assert gazebo_names == expected_gazebo_names
    assert sorted(isaac_configs) == sorted(expected_isaac_names)
    assert sorted(gazebo_configs) == sorted(expected_gazebo_names)


def test_table_camera_yaml_wires_stamped_bbox_without_legacy_stereo_input():
    config_path = (
        _PACKAGE_ROOT
        / 'config'
        / 'perception'
        / 'table_camera_perception.yaml'
    )
    config = yaml.safe_load(config_path.read_text())

    stereo_params = config['stereo_sgbm_pointcloud']['ros__parameters']
    ess_params = config['ess_depth_pointcloud']['ros__parameters']
    foundationpose_adapter_params = config[
        'foundationpose_detection_to_pose'
    ]['ros__parameters']
    megapose_params = config['megapose_detection']['ros__parameters']
    megapose_adapter_params = config[
        'megapose_detection_to_pose'
    ]['ros__parameters']
    yolo_params = config['yolo_visualizer']['ros__parameters']
    frame_fix_params = config['ess_disparity_frame_fix']['ros__parameters']

    assert 'detection_bbox_topic' not in stereo_params
    assert 'detection_bbox_topic' not in ess_params
    assert yolo_params['detection_bbox_topic'] == '$(var detection_bbox_topic)'
    assert yolo_params['detection_bbox_stamped_topic'] == '$(var detection_bbox_stamped_topic)'
    assert yolo_params['detection_bbox_frame_id'] == 'table_stereo_left_optical_frame'
    assert frame_fix_params['force_frame_id'] == yolo_params['detection_bbox_frame_id']
    assert stereo_params['detection_bbox_stamped_topic'] == '$(var detection_bbox_stamped_topic)'
    assert ess_params['detection_bbox_stamped_topic'] == '$(var detection_bbox_stamped_topic)'
    assert stereo_params['allow_latest_tf_fallback'] is True
    assert ess_params['allow_latest_tf_fallback'] is True
    assert megapose_params['detection_bbox_stamped_topic'] == '$(var detection_bbox_stamped_topic)'
    assert megapose_params['gigapose_checkpoint_path'] == (
        '$(var gigapose_checkpoint_path)'
    )
    assert megapose_params['gigapose_min_matching_score'] == (
        '$(var gigapose_min_matching_score)'
    )
    assert stereo_params['left_image_frame_id'] == 'table_stereo_left_optical_frame'
    assert stereo_params['right_image_frame_id'] == 'table_stereo_right_optical_frame'
    assert stereo_params['table_plane_marker_topic'] == '/table_stereo/table_plane'
    assert ess_params['table_plane_marker_topic'] == '/table_stereo/table_plane'

    expected_adapter_params = {
        'output_pose_topic': '/table_stereo/cube_pose',
        'output_marker_topic': '/table_stereo/cube_geometry',
        'marker_namespace': 'cube_geometry',
        'fallback_object_size_m': 0.05,
    }
    legacy_adapter_params = {
        'cube_pose_topic',
        'cube_marker_topic',
        'fixed_cube_size_m',
    }
    for adapter_params in (
        foundationpose_adapter_params,
        megapose_adapter_params,
    ):
        for parameter_name, expected_value in expected_adapter_params.items():
            assert adapter_params[parameter_name] == expected_value
        assert legacy_adapter_params.isdisjoint(adapter_params)


def test_launch_helper_modules_document_responsibility_and_non_ownership():
    for path in sorted((_PACKAGE_ROOT / 'launch' / 'franka_motion_parts').rglob('*.py')):
        module = ast.parse(path.read_text())
        docstring = ast.get_docstring(module, clean=False) or ''

        assert 'Responsibility:' in docstring, path
        assert 'Does not own:' in docstring, path


def test_gazebo_launch_tests_allow_cold_start_budget():
    assert DEFAULT_RUNTIME_TIMEOUT >= 75.0


def test_pipeline_conditions_route_modes_to_expected_backends():
    conditions = create_pipeline_conditions({'mode': LaunchConfiguration('mode')})

    assert _condition_value(conditions['sgbm'], 'none')
    assert _condition_value(conditions['sgbm'], 'ecc')
    assert not _condition_value(conditions['ess'], 'none')
    assert _condition_value(conditions['ess'], 'gigaPose')
    assert _condition_value(conditions['megapose'], 'auto')
    assert _condition_value(conditions['megapose'], 'megaPose')
    assert _condition_value(conditions['megapose'], 'gigaPose')
    assert not _condition_value(conditions['foundationpose'], 'gigaPose')
    assert _condition_value(conditions['foundationpose'], 'foundationPose')
    assert _condition_value(conditions['pose_estimator_input'], 'foundationPose')
    assert _condition_value(conditions['yolo'], 'yolo')
    assert _condition_value(conditions['yolo'], 'gigaPose')
    assert not _condition_value(conditions['yolo'], 'none')


def test_perception_node_factories_target_canonical_package(monkeypatch):
    """Launch integration must bypass the legacy executable facade."""
    def node_factory(**kwargs):
        return kwargs

    for module in (pose_estimators, stereo_depth, yolo):
        monkeypatch.setattr(module, 'Node', node_factory)
        monkeypatch.setattr(
            module,
            'table_camera_parameters',
            lambda *_args, **_kwargs: [],
        )

    mode = LaunchConfiguration('mode')
    conditions = create_pipeline_conditions({'mode': mode})
    configs = {
        'mode': mode,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yolo_class_whitelist': LaunchConfiguration('yolo_class_whitelist'),
        'yolo_python': LaunchConfiguration('yolo_python'),
    }
    paths = SimpleNamespace(managed_process_wrapper=Path('/pkg/managed_wrapper.py'))

    sgbm = stereo_depth.create_sgbm_node(configs, conditions)
    ess = stereo_depth.create_ess_actions(paths, configs, conditions)
    pose_input = pose_estimators.create_pose_estimator_input_nodes(configs, conditions)
    foundationpose = pose_estimators.create_foundationpose_actions(paths, configs, conditions)
    megapose = pose_estimators.create_megapose_actions(configs, conditions)
    yolo_node = yolo.create_yolo_node(paths.managed_process_wrapper, configs, conditions)

    common_nodes = (
        sgbm,
        ess.disparity_frame_fix,
        ess.depth_pointcloud,
    )
    pose_nodes = (
        pose_input.frame_fix,
        pose_input.depth_clean,
        foundationpose.pose_adapter,
        megapose.detection,
        megapose.pose_adapter,
    )
    assert PERCEPTION_PACKAGE == 'manipulation_perception'
    assert all(node['package'] == PERCEPTION_PACKAGE for node in common_nodes)
    assert all(node['package'] == PERCEPTION_POSE_PACKAGE for node in pose_nodes)
    assert yolo_node['package'] == PERCEPTION_YOLO_PACKAGE


def test_camera_pipeline_action_groups_preserve_launch_order():
    perception_actions = PerceptionActions(
        image_bridges=ImageBridgeActions(left='left_bridge', right='right_bridge'),
        stereo_pointcloud='stereo_pointcloud',
        ess=EssActions(
            disparity_frame_fix='disparity_frame_fix',
            disparity_container='disparity_container',
            depth_pointcloud='ess_depth_pointcloud',
        ),
        pose_input=PoseEstimatorInputActions(
            frame_fix='pose_input_frame_fix',
            depth_clean='pose_input_depth_clean',
        ),
        foundationpose=FoundationPoseActions(
            container='foundationpose_container',
            pose_adapter='foundationpose_pose_adapter',
        ),
        megapose=MegaPoseActions(
            detection='megapose_detection',
            pose_adapter='megapose_pose_adapter',
        ),
        yolo_visualizer='yolo_visualizer',
    )
    camera_tf_actions = CameraTfActions(
        left='left_tf',
        right='right_tf',
        left_optical='left_optical_tf',
        right_optical='right_optical_tf',
    )
    rviz_actions = RvizActions(node='rviz', delayed_start='delayed_rviz')

    expected_common_actions = [
        'stereo_pointcloud',
        'disparity_frame_fix',
        'disparity_container',
        'ess_depth_pointcloud',
        'yolo_visualizer',
        'left_tf',
        'right_tf',
        'left_optical_tf',
        'right_optical_tf',
        'pose_input_frame_fix',
        'pose_input_depth_clean',
        'foundationpose_container',
        'foundationpose_pose_adapter',
        'megapose_detection',
        'megapose_pose_adapter',
        'delayed_rviz',
    ]
    gazebo_actions = create_camera_pipeline_actions(
        perception_actions,
        camera_tf_actions,
        rviz_actions,
        include_image_bridges=True,
    )
    isaac_actions = create_camera_pipeline_actions(
        perception_actions,
        camera_tf_actions,
        rviz_actions,
        include_image_bridges=False,
    )

    assert gazebo_actions == ['left_bridge', 'right_bridge', *expected_common_actions]
    assert isaac_actions == expected_common_actions


def test_create_table_camera_paths_maps_canonical_resource_owners(monkeypatch):
    paths = _canonical_camera_paths(
        bringup_share='/opt/share/franka_motion',
    )

    assert paths.camera_sdf == Path(
        '/opt/share/franka_motion/description/gazebo/table_camera.sdf'
    )
    assert paths.rviz_config == Path(
        '/opt/share/franka_motion/config/rviz/'
        'franka_table_stereo_pointcloud.rviz'
    )
    assert paths.perception_config == Path(
        '/opt/share/franka_motion/config/perception/'
        'table_camera_perception.yaml'
    )
    assert paths.yolo_model == Path(
        '/opt/share/franka_motion/models/cube_yolo26s_isaac_seg_v5s.pt'
    )
    assert paths.foundationpose_mesh == Path(
        '/opt/share/franka_motion/models/foundationpose/cube_5cm.obj'
    )
    assert paths.foundationpose_texture == Path(
        '/opt/share/franka_motion/models/foundationpose/cube_5cm_texture.ppm'
    )


def test_table_camera_path_uses_canonical_simulator_owner(monkeypatch):
    paths = _canonical_camera_paths()

    assert paths.camera_sdf == Path(
        '/share/franka_motion/description/gazebo/table_camera.sdf'
    )
    assert paths.perception_config == Path(
        '/share/franka_motion/config/perception/table_camera_perception.yaml'
    )


def test_image_bridge_uses_managed_kill_shutdown_prefix():
    prefix = image_bridge_shutdown_prefix(
        Path('/pkg/lib/franka_motion/managed_process_wrapper.py'),
        'table_stereo_right_image_bridge',
    )

    assert prefix == (
        '/pkg/lib/franka_motion/managed_process_wrapper.py '
        '--child-name table_stereo_right_image_bridge --shutdown-signal kill --'
    )


def test_yolo_uses_managed_shutdown_prefix_around_configured_python():
    context = LaunchContext()
    context.launch_configurations['yolo_python'] = '/venv/yolo/bin/python'

    prefix = yolo_shutdown_prefix(
        Path('/pkg/lib/franka_motion/managed_process_wrapper.py'),
        LaunchConfiguration('yolo_python'),
    )

    assert perform_substitutions(context, prefix) == (
        '/pkg/lib/franka_motion/managed_process_wrapper.py '
        '--child-name yolo_visualizer '
        '--shutdown-signal int '
        '--shutdown-timeout-sec 0.8 '
        '--kill-timeout-sec 1.0 -- '
        '/venv/yolo/bin/python'
    )


def test_table_camera_validation_skips_backend_checks_for_none_mode():
    context = LaunchContext()
    context.launch_configurations['mode'] = 'none'

    for action in create_table_camera_validation_actions():
        assert action.execute(context) == []
