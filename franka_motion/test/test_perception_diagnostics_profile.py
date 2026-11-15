"""
Validate perception-diagnostic bindings in the table-camera profile.

Responsibility: Ensure alternative stereo producers map to the exact monitored diagnostics.
"""

from pathlib import Path

from manipulation_msgs.msg import HealthComponentStatus
import yaml


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_table_camera_profile_binds_alternative_stereo_producers():
    config_path = (
        _PACKAGE_ROOT / 'config' / 'perception' /
        'table_camera_perception.yaml'
    )
    config = yaml.safe_load(config_path.read_text())
    health_params = config['system_health_monitor']['ros__parameters']

    allowed_tasks = health_params['perception_diagnostics.allowed_tasks']
    component_ids = health_params['perception_diagnostics.component_ids']
    status_names = health_params['perception_diagnostics.status_names']

    assert allowed_tasks == [
        'perception/stereo_depth_pointcloud',
        'perception/stereo_depth_pointcloud',
        'perception/yolo_detection',
        'perception/pose_estimation',
        'perception/pose_estimation',
    ]
    assert component_ids == [
        HealthComponentStatus.COMPONENT_STEREO_PERCEPTION,
        HealthComponentStatus.COMPONENT_STEREO_PERCEPTION,
        HealthComponentStatus.COMPONENT_OBJECT_DETECTION,
        HealthComponentStatus.COMPONENT_POSE_ESTIMATION,
        HealthComponentStatus.COMPONENT_POSE_ESTIMATION,
    ]
    assert status_names == [
        'stereo_sgbm_pointcloud: perception/stereo_depth_pointcloud',
        'ess_depth_pointcloud: perception/stereo_depth_pointcloud',
        'yolo_visualizer: perception/yolo_detection',
        'foundationpose_detection_to_pose: perception/pose_estimation',
        'megapose_detection_to_pose: perception/pose_estimation',
    ]
    assert len(allowed_tasks) == len(component_ids) == len(status_names)
    # Object visibility is a task-specific precondition. The detected-object
    # resolver requires a fresh stamped pose before every grasp, while global
    # motion readiness remains usable during expected grasp occlusion.
    assert 'perception_diagnostics.required_tasks' not in health_params
