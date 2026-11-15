"""
MoveIt configuration and move_group launch node factories.

Responsibility:
- Build Gazebo and Isaac MoveIt configurations from package resources.
- Select one explicit occupancy-map profile supplied by the scenario leaf.
- Create move_group nodes with simulator-appropriate robot descriptions and parameters.
- Create the exact-stamp TF gate used by the table-camera Octomap profile.

Does not own:
- Planning pipeline tuning files from the MoveIt config package.
- Controller startup, sensor-cloud production or perception pipeline wiring.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from franka_motion_runtime.bringup.contracts import (
    InvalidBringupSelection,
    PlannerBackend,
)
from franka_motion_runtime.bringup.launch_parts.constants import (
    MOVEIT_BACKEND_PACKAGE,
    MOVEIT_CONFIG_PACKAGE,
    ROBOT_NAME,
)
from franka_motion_runtime.bringup.launch_parts.descriptions import (
    generate_isaac_robot_description,
)
from franka_motion_runtime.bringup.launch_parts.paths import GazeboLaunchPaths, IsaacLaunchPaths


OCTOMAP_TF_REQUIRED_FRAMES = ("panda_leftfinger", "panda_rightfinger")
_CUROBO_PACKAGE = "manipulation_curobo"
_CUROBO_PLUGIN_LIBRARY = "libcurobo_moveit_plugin.so"
_EXACT_SCENE_CAPABILITY = (
    "manipulation_moveit/ExactPlanningSceneMoveGroupCapability"
)
_CANCELABLE_EXECUTION_CAPABILITY = (
    "manipulation_moveit/CancelableExecuteTrajectoryCapability"
)
_UPSTREAM_MOVE_ACTION_CAPABILITY = "move_group/MoveGroupMoveAction"
_UPSTREAM_EXECUTE_ACTION_CAPABILITY = "move_group/MoveGroupExecuteTrajectoryAction"


class OctomapProfile(Enum):
    """MoveIt occupancy-map profile selected by the bringup scenario."""

    DISABLED = "config/moveit/sensors_3d_disabled.yaml"
    TABLE_CAMERA = "config/moveit/sensors_3d_table_camera.yaml"


def declare_planner_backend_argument() -> DeclareLaunchArgument:
    """Declare the backend profile shared by all maintained MoveIt leaves."""
    return DeclareLaunchArgument(
        "planner_backend",
        default_value=PlannerBackend.MOVEIT.value,
        choices=[backend.value for backend in PlannerBackend],
        description=(
            "MoveIt planning pipeline: moveit (OMPL) or optional curobo."
        ),
    )


def resolve_planner_backend(
    context: LaunchContext,
    *,
    octomap_profile: OctomapProfile,
) -> PlannerBackend:
    """Resolve and validate a backend before constructing any ROS process."""
    backend = PlannerBackend.parse(
        LaunchConfiguration("planner_backend").perform(context)
    )
    validate_planner_backend(backend, octomap_profile=octomap_profile)
    return backend


def validate_planner_backend(
    backend: PlannerBackend,
    *,
    octomap_profile: OctomapProfile,
) -> None:
    """Fail early for unsupported scenes or an unavailable optional plugin."""
    if backend is PlannerBackend.MOVEIT:
        return
    if octomap_profile is OctomapProfile.TABLE_CAMERA:
        raise InvalidBringupSelection(
            "planner_backend='curobo' cannot represent the table-camera "
            "Octomap; select planner_backend='moveit'"
        )

    try:
        package_share = Path(get_package_share_directory(_CUROBO_PACKAGE))
        package_prefix = Path(get_package_prefix(_CUROBO_PACKAGE))
    except PackageNotFoundError as error:
        raise RuntimeError(
            "planner_backend='curobo' requires the optional "
            "manipulation_curobo package"
        ) from error

    required_resources = (
        package_share / "curobo_moveit_plugin_description.xml",
        package_prefix / "lib" / _CUROBO_PLUGIN_LIBRARY,
        package_prefix / "share" / "ament_index" / "resource_index" /
        "moveit_core__pluginlib__plugin" / _CUROBO_PACKAGE,
    )
    missing = [str(path) for path in required_resources if not path.is_file()]
    if missing:
        raise RuntimeError(
            "planner_backend='curobo' is installed incompletely; missing: "
            + ", ".join(missing)
        )


def planning_pipeline_name(backend: PlannerBackend) -> str:
    """Map a user-facing backend profile to the MoveIt pipeline namespace."""
    return "ompl" if backend is PlannerBackend.MOVEIT else "curobo"


def manipulation_move_group_capabilities() -> dict[str, str]:
    """Keep one owner for exact planning and cancelable trajectory execution."""
    return {
        "capabilities": f"{_EXACT_SCENE_CAPABILITY} {_CANCELABLE_EXECUTION_CAPABILITY}",
        "disable_capabilities": (
            f"{_UPSTREAM_MOVE_ACTION_CAPABILITY} {_UPSTREAM_EXECUTE_ACTION_CAPABILITY}"
        ),
    }


def create_octomap_tf_gate_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
) -> Node:
    """Admit sensor clouds only after the complete Panda TF chain is ready."""
    return Node(
        package=MOVEIT_BACKEND_PACKAGE,
        executable="moveit_octomap_tf_gate",
        name="moveit_octomap_tf_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/table_stereo/octomap_points",
                "output_topic": "/table_stereo/moveit_octomap_points",
                "expected_source_frame": "table_stereo_left_optical_frame",
                # Both leaf transforms require every dynamic arm link plus the
                # fixed hand transform, and cover the two independent fingers.
                "required_frames": list(OCTOMAP_TF_REQUIRED_FRAMES),
                "queue_size": 30,
                "transform_timeout_sec": 0.5,
                "health_timeout_sec": 1.0,
            }
        ],
    )


def create_moveit_servo_node(
    *,
    use_sim_time: LaunchConfiguration | bool,
    start_servo: LaunchConfiguration,
    moveit_config,
    servo_config_path: Path,
) -> Node:
    """Create the optional Jazzy MoveIt Servo ROS node for Panda control."""
    servo_parameters = yaml.safe_load(
        servo_config_path.read_text(encoding="utf-8")
    )
    if not isinstance(servo_parameters, dict):
        raise ValueError(
            f"MoveIt Servo config must contain a parameter map: "
            f"{servo_config_path}"
        )

    # The acceleration-limited smoother uses these two root-level parameters;
    # Servo itself reads the remaining values below the moveit_servo prefix.
    smoothing_parameters = {
        "update_period": servo_parameters["publish_period"],
        "planning_group_name": servo_parameters["move_group_name"],
    }
    return Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        condition=IfCondition(start_servo),
        parameters=[
            {"moveit_servo": servo_parameters},
            smoothing_parameters,
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )


def create_gazebo_moveit_config(
    paths: GazeboLaunchPaths,
    robot_description_xml: str,
    *,
    octomap_profile: OctomapProfile,
    planner_backend: PlannerBackend = PlannerBackend.MOVEIT,
):
    """Build the Gazebo MoveIt configuration with the generated robot model."""
    validate_planner_backend(
        planner_backend,
        octomap_profile=octomap_profile,
    )
    planning_pipeline = planning_pipeline_name(planner_backend)
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PACKAGE)
        .robot_description(file_path="config/moveit/panda.urdf.xacro")
        .robot_description_semantic(file_path=str(paths.semantic_config))
        .robot_description_kinematics(
            file_path="config/moveit/kinematics.yaml"
        )
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline=planning_pipeline,
            pipelines=[planning_pipeline],
        )
        .trajectory_execution(
            file_path="config/moveit/moveit_controllers.yaml"
        )
        .planning_scene_monitor()
        .sensors_3d(file_path=octomap_profile.value)
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description_xml}
    moveit_config.robot_description_kinematics = _load_kinematics(paths.kinematics_config)
    return moveit_config


def create_isaac_moveit_config(
    paths: IsaacLaunchPaths,
    *,
    octomap_profile: OctomapProfile,
    planner_backend: PlannerBackend = PlannerBackend.MOVEIT,
):
    """Build the Isaac MoveIt configuration for the selected scene profile."""
    validate_planner_backend(
        planner_backend,
        octomap_profile=octomap_profile,
    )
    planning_pipeline = planning_pipeline_name(planner_backend)
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PACKAGE)
        .robot_description(file_path="config/moveit/panda.urdf.xacro")
        .robot_description_semantic(file_path=str(paths.semantic_config))
        .robot_description_kinematics(
            file_path="config/moveit/kinematics.yaml"
        )
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline=planning_pipeline,
            pipelines=[planning_pipeline],
        )
        .trajectory_execution(
            file_path="config/moveit/moveit_controllers.yaml"
        )
        .planning_scene_monitor()
        .sensors_3d(file_path=octomap_profile.value)
        .to_moveit_configs()
    )
    moveit_config.robot_description_kinematics = _load_kinematics(paths.kinematics_config)
    moveit_config.robot_description = {
        "robot_description": generate_isaac_robot_description(paths),
    }
    return moveit_config


def create_gazebo_move_group_node(
    *,
    use_sim_time: LaunchConfiguration,
    moveit_config,
    managed_process_wrapper_path: Path,
    output: str = "screen",
    ros_arguments: list[str] | None = None,
) -> Node:
    """Create a managed Gazebo move_group process with simulation time."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output=output,
        prefix=f"{managed_process_wrapper_path} --child-name move_group --shutdown-signal kill --",
        ros_arguments=ros_arguments,
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.to_dict(),
            manipulation_move_group_capabilities(),
        ],
    )


def create_isaac_move_group_node(
    *,
    paths: IsaacLaunchPaths,
    use_sim_time: bool,
    moveit_config,
) -> Node:
    """Create a managed Isaac move_group process with controller overrides."""
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        prefix=(
            f"{paths.managed_process_wrapper} --child-name move_group "
            "--shutdown-signal kill --"
        ),
        parameters=[
            moveit_config.to_dict(),
            ParameterFile(str(paths.moveit_trajectory_execution_config), allow_substs=True),
            {"use_sim_time": use_sim_time},
            manipulation_move_group_capabilities(),
        ],
    )


def _load_kinematics(kinematics_config: Path) -> dict[str, object]:
    return {
        "robot_description_kinematics": yaml.safe_load(kinematics_config.read_text())
    }
