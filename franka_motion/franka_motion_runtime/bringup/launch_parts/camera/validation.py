"""
Runtime validation for table-camera mode-specific dependencies.

Responsibility:
- Fail early when selected table-camera modes miss required packages or files.
- Validate backend-specific paths for ESS, FoundationPose, MegaPose and GigaPose.
- Keep optional perception dependencies fail-fast.

Does not own:
- Declaring launch arguments or selecting mode conditions.
- Runtime recovery when a launched backend process fails.
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from manipulation_perception.pose.pose_gigapose_parameters import (
    resolve_gigapose_checkpoint,
)

from franka_motion_runtime.bringup.launch_parts.camera.constants import ESS_MODES, VALID_MODES
from franka_motion_runtime.bringup.launch_parts.camera.modes import _invalid_mode_message


ISAAC_ROS_BUNDLE = "manipulation_perception_isaac_ros"


def _require_isaac_ros_bundle(mode_name: str) -> None:
    """Require the versioned optional integration before checking components."""
    try:
        get_package_share_directory(ISAAC_ROS_BUNDLE)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f"mode:={mode_name} требует optional package {ISAAC_ROS_BUNDLE}. "
            "Соберите его на host с установленным NVIDIA Isaac ROS 4.1."
        ) from exc


def _validate_mode(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode_value not in VALID_MODES:
        raise RuntimeError(_invalid_mode_message(mode_value))
    return []


def _validate_foundationpose_setup(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode_value != "foundationpose":
        return []

    _require_isaac_ros_bundle(mode_value)

    try:
        get_package_share_directory("isaac_ros_foundationpose")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "mode:=foundationPose требует установленного пакета "
            "ros-jazzy-isaac-ros-foundationpose."
        ) from exc

    refine_model = LaunchConfiguration("foundationpose_refine_model_file_path").perform(context).strip()
    score_model = LaunchConfiguration("foundationpose_score_model_file_path").perform(context).strip()
    refine_engine = LaunchConfiguration("foundationpose_refine_engine_file_path").perform(context).strip()
    score_engine = LaunchConfiguration("foundationpose_score_engine_file_path").perform(context).strip()
    mesh_file = LaunchConfiguration("foundationpose_mesh_file_path").perform(context).strip()
    texture_file = LaunchConfiguration("foundationpose_texture_path").perform(context).strip()

    for label, file_path in [
        ("FoundationPose refine model", refine_model),
        ("FoundationPose score model", score_model),
        ("FoundationPose refine engine", refine_engine),
        ("FoundationPose score engine", score_engine),
        ("FoundationPose mesh", mesh_file),
    ]:
        if not file_path or not Path(file_path).is_file():
            raise RuntimeError(f"{label} не найден: {file_path}")

    if texture_file and not Path(texture_file).is_file():
        raise RuntimeError(f"FoundationPose texture не найдена: {texture_file}")

    return []


def _validate_ess_setup(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode_value not in ESS_MODES:
        return []

    _require_isaac_ros_bundle(mode_value)

    for package_name in (
        "isaac_ros_ess",
        "isaac_ros_image_proc",
        "isaac_ros_stereo_image_proc",
    ):
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "mode:=ess, mode:=yolo, mode:=foundationPose, mode:=megaPose и mode:=gigaPose "
                "требуют установленных пакетов Isaac ROS ESS. "
                "Установите зависимости и пересоберите workspace."
            ) from exc

    ess_engine_file_path = LaunchConfiguration("ess_engine_file_path").perform(context).strip()
    if not ess_engine_file_path:
        raise RuntimeError(
            "Для ESS-ветки укажите путь к TensorRT-движку: "
            "ess_engine_file_path:=/abs/path/to/ess.engine"
        )
    if not Path(ess_engine_file_path).is_file():
        raise RuntimeError(
            f"ESS engine не найден: {ess_engine_file_path}. "
            "Проверьте путь к файлу ess.engine."
        )

    return []


def _validate_megapose_setup(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode_value not in {"megapose", "auto", "gigapose"}:
        return []

    mesh_file = LaunchConfiguration("megapose_mesh_file_path").perform(context).strip()
    if not mesh_file or not Path(mesh_file).is_file():
        raise RuntimeError(f"MegaPose mesh не найден: {mesh_file}")

    megapose_python = LaunchConfiguration("megapose_python").perform(context).strip()
    if "/" in megapose_python and not Path(megapose_python).is_file():
        raise RuntimeError(f"MegaPose python не найден: {megapose_python}")

    return []


def _validate_gigapose_setup(context, *args, **kwargs):
    mode_value = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode_value != "gigapose":
        return []

    megapose_data_dir = Path(LaunchConfiguration("megapose_data_dir").perform(context).strip())
    gigapose_python = LaunchConfiguration("gigapose_python").perform(context).strip()
    gigapose_repo_dir = Path(LaunchConfiguration("gigapose_repo_dir").perform(context).strip())
    dinov2_repo_dir = LaunchConfiguration("gigapose_dinov2_repo_dir").perform(context).strip()
    gigapose_data_dir = Path(LaunchConfiguration("gigapose_data_dir").perform(context).strip())
    gigapose_dataset_name = LaunchConfiguration("gigapose_dataset_name").perform(context).strip()
    gigapose_object_id = int(LaunchConfiguration("gigapose_object_id").perform(context))
    checkpoint_path = resolve_gigapose_checkpoint(
        gigapose_data_dir,
        LaunchConfiguration("gigapose_checkpoint_path").perform(context),
    )

    if "/" in gigapose_python and not Path(gigapose_python).is_file():
        raise RuntimeError(f"GigaPose python не найден: {gigapose_python}")
    if not (gigapose_repo_dir / "configs" / "test.yaml").is_file():
        raise RuntimeError(f"GigaPose repo не найден или неполный: {gigapose_repo_dir}")
    if dinov2_repo_dir and not (Path(dinov2_repo_dir) / "hubconf.py").is_file():
        raise RuntimeError(f"DINOv2 repo не найден или неполный: {dinov2_repo_dir}")
    if not checkpoint_path.is_file():
        raise RuntimeError(f"GigaPose checkpoint не найден: {checkpoint_path}")
    if gigapose_object_id <= 0:
        raise RuntimeError("gigapose_object_id должен быть положительным BOP object id")

    template_root = gigapose_data_dir / "datasets" / "templates" / gigapose_dataset_name
    template_dir = template_root / f"{gigapose_object_id:06d}"
    pose_path = template_root / "object_poses" / f"{gigapose_object_id:06d}.npy"
    models_info_path = gigapose_data_dir / "datasets" / gigapose_dataset_name / "models" / "models_info.json"
    if not template_dir.is_dir() or not pose_path.is_file():
        raise RuntimeError(
            "GigaPose templates не найдены: "
            f"ожидались {template_dir} и {pose_path}"
        )
    if not models_info_path.is_file():
        raise RuntimeError(f"GigaPose models_info.json не найден: {models_info_path}")

    megapose_models_root = megapose_data_dir / "megapose-models"
    for run_dir in ("coarse-rgb-906902141", "refiner-rgb-653307694"):
        checkpoint_file = megapose_models_root / run_dir / "checkpoint.pth.tar"
        if not checkpoint_file.is_file():
            raise RuntimeError(f"MegaPose checkpoint для GigaPose refiner не найден: {checkpoint_file}")

    return []


def create_table_camera_validation_actions() -> list[OpaqueFunction]:
    """Create fail-fast validation for the selected optional backends."""
    actions = [
        OpaqueFunction(function=_validate_mode),
        OpaqueFunction(function=_validate_ess_setup),
        OpaqueFunction(function=_validate_megapose_setup),
        OpaqueFunction(function=_validate_gigapose_setup),
        OpaqueFunction(function=_validate_foundationpose_setup),
    ]
    return actions
