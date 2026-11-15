"""
Factory for pose backend runners used by `megapose_detection_node`.

Responsibility:
- Wire ROS node parameters into MegaPose and GigaPose backend runners.
- Create worker clients, runtime file adapters and backend configs.
- Keep the node constructor free from backend-specific process details.

Does not own:
- ROS subscriptions, publishers, CvBridge or message headers.
- Backend inference logic; those belong to `pose_megapose_backend` and
  `pose_gigapose_pipeline`.
- Worker subprocess protocol internals; those belong to `pose_worker_client`.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from manipulation_perception.pose.pose_backend_common import LogCallback
from manipulation_perception.pose.pose_gigapose_pipeline import (
    GigaPosePipelineRunner,
    GigaPoseRefinerRequestConfig,
    GigaPoseWorkerRequestConfig,
)
from manipulation_perception.pose.pose_megapose_backend import (
    MegaPoseBackendRunner,
    MegaPoseCliConfig,
    MegaPoseWorkerRequestConfig,
)
from manipulation_perception.pose.pose_node_parameters import PoseNodeParameters
from manipulation_perception.pose.pose_runtime_environment import (
    make_gigapose_env,
    make_megapose_env,
    resolve_worker_script_path,
)
from manipulation_perception.pose.pose_runtime_files import (
    GigaPoseRuntimeInput,
    MegaPoseRuntimeExample,
    PoseRuntimeWorkspace,
)
from manipulation_perception.pose.pose_worker_client import JsonWorkerClient


@dataclass(frozen=True)
class PoseBackendRunners:
    """
    Owned MegaPose and GigaPose runners for one pose-estimation node.

    Each runner owns worker-process clients; the node must stop both runners
    before destroying ROS resources.
    """

    megapose: MegaPoseBackendRunner
    gigapose: GigaPosePipelineRunner
    workspace: PoseRuntimeWorkspace

    def cleanup(self) -> bool:
        """Release runtime files only after the caller has joined all writers."""
        owners = (
            self.megapose,
            self.megapose.worker_client,
            self.gigapose.worker_client,
            self.gigapose.refiner_worker_client,
        )
        if any(owner.has_cleanup_failure for owner in owners):
            return False
        self.workspace.cleanup()
        return True


def create_pose_backend_runners(
    params: PoseNodeParameters,
    current_file: Path,
    *,
    log_info: LogCallback,
    log_warn: LogCallback,
    log_error: LogCallback,
    log_warn_throttled: LogCallback | None = None,
    on_worker_restart: Callable[[], None] | None = None,
    on_worker_timeout: Callable[[], None] | None = None,
) -> PoseBackendRunners:
    """
    Create backend runners and their worker-process ownership graph.

    Worker commands inherit validated parameter snapshots and dedicated runtime
    environments. No process is started until a runner receives prewarm or
    inference work; shutdown remains the caller's responsibility via each
    runner's ``stop``.
    """
    common = params.common
    megapose = params.megapose
    gigapose = params.gigapose
    workspace = PoseRuntimeWorkspace(megapose.megapose_data_dir, gigapose.gigapose_data_dir)
    try:
        runtime_example = MegaPoseRuntimeExample(
            workspace.megapose_data_dir / "examples" / params.example_name,
            megapose.object_label,
            megapose.mesh_file_path,
            megapose.mesh_units,
        )
        gigapose_runtime_input = GigaPoseRuntimeInput(
            workspace.gigapose_data_dir / "ros_runtime" / "inputs" / megapose.object_label
        )

        def gigapose_env() -> dict[str, str]:
            workspace.prepare_gigapose_cache()
            return make_gigapose_env(
                workspace.gigapose_data_dir, gigapose.gigapose_repo_dir, gigapose.gigapose_python,
            )

        megapose_worker_client = JsonWorkerClient(
            worker_name="MegaPose",
            python_path=megapose.megapose_python,
            worker_script=resolve_worker_script_path(current_file, "megapose_worker.py"),
            env_factory=lambda: make_megapose_env(
                workspace.megapose_data_dir,
                megapose.megapose_python,
            ),
            timeout_sec=common.process_timeout_sec,
            persistent=megapose.persistent_worker,
            verbose=common.verbose_subprocess,
            log_info=log_info,
            on_worker_restart=on_worker_restart,
        )
        gigapose_worker_client = JsonWorkerClient(
            worker_name="GigaPose",
            python_path=gigapose.gigapose_python,
            worker_script=resolve_worker_script_path(current_file, "gigapose_worker.py"),
            env_factory=gigapose_env,
            timeout_sec=common.process_timeout_sec,
            persistent=gigapose.gigapose_persistent_worker,
            verbose=common.verbose_subprocess,
            log_info=log_info,
            on_worker_restart=on_worker_restart,
        )
        gigapose_refiner_worker_client = JsonWorkerClient(
            worker_name="GigaPose refiner",
            python_path=gigapose.gigapose_python,
            worker_script=resolve_worker_script_path(current_file, "gigapose_refiner_worker.py"),
            env_factory=gigapose_env,
            timeout_sec=common.process_timeout_sec,
            persistent=True,
            verbose=common.verbose_subprocess,
            log_info=log_info,
            shutdown_command={"command": "shutdown"},
            on_worker_restart=on_worker_restart,
        )

        return PoseBackendRunners(
            megapose=_create_megapose_runner(
                params=params,
                data_dir=workspace.megapose_data_dir,
                runtime_example=runtime_example,
                worker_client=megapose_worker_client,
                log_info=log_info,
                log_warn=log_warn,
                log_error=log_error,
                on_worker_timeout=on_worker_timeout,
            ),
            gigapose=_create_gigapose_runner(
                params=params,
                data_dir=workspace.gigapose_data_dir,
                runtime_input=gigapose_runtime_input,
                worker_client=gigapose_worker_client,
                refiner_worker_client=gigapose_refiner_worker_client,
                log_info=log_info,
                log_warn=log_warn_throttled or log_warn,
            ),
            workspace=workspace,
        )
    except Exception:
        workspace.cleanup()
        raise


def _create_megapose_runner(
    *,
    params: PoseNodeParameters,
    data_dir: Path,
    runtime_example: MegaPoseRuntimeExample,
    worker_client: JsonWorkerClient,
    log_info: LogCallback,
    log_warn: LogCallback,
    log_error: LogCallback,
    on_worker_timeout: Callable[[], None] | None,
) -> MegaPoseBackendRunner:
    common = params.common
    megapose = params.megapose
    return MegaPoseBackendRunner(
        runtime_example=runtime_example,
        worker_client=worker_client,
        worker_config=MegaPoseWorkerRequestConfig(
            example_name=params.example_name,
            example_dir=runtime_example.example_dir,
            object_label=megapose.object_label,
            bsz_images=megapose.bsz_images,
            bsz_objects=megapose.bsz_objects,
            renderer_workers=megapose.renderer_workers,
        ),
        cli_config=MegaPoseCliConfig(
            python_path=megapose.megapose_python,
            example_name=params.example_name,
            output_pose_paths=runtime_example.output_pose_paths,
            object_label=megapose.object_label,
            process_timeout_sec=common.process_timeout_sec,
            env_factory=lambda: make_megapose_env(
                data_dir,
                megapose.megapose_python,
            ),
        ),
        use_worker=megapose.persistent_worker,
        verbose=common.verbose_subprocess,
        log_info=log_info,
        log_warn=log_warn,
        log_error=log_error,
        on_worker_timeout=on_worker_timeout,
    )


def _create_gigapose_runner(
    *,
    params: PoseNodeParameters,
    data_dir: Path,
    runtime_input: GigaPoseRuntimeInput,
    worker_client: JsonWorkerClient,
    refiner_worker_client: JsonWorkerClient,
    log_info: LogCallback,
    log_warn: LogCallback,
) -> GigaPosePipelineRunner:
    common = params.common
    megapose = params.megapose
    gigapose = params.gigapose
    return GigaPosePipelineRunner(
        runtime_input=runtime_input,
        worker_client=worker_client,
        refiner_worker_client=refiner_worker_client,
        worker_config=GigaPoseWorkerRequestConfig(
            repo_dir=gigapose.gigapose_repo_dir,
            dinov2_repo_dir=gigapose.gigapose_dinov2_repo_dir,
            root_dir=data_dir,
            dataset_name=gigapose.gigapose_dataset_name,
            object_id=gigapose.gigapose_object_id,
            checkpoint_path=gigapose.gigapose_checkpoint_path,
            template_count=gigapose.gigapose_template_count,
            max_num_dets_per_forward=gigapose.gigapose_max_num_dets_per_forward,
            translation_scale=gigapose.gigapose_translation_scale,
            num_hypotheses=gigapose.gigapose_num_hypotheses,
        ),
        refiner_config=GigaPoseRefinerRequestConfig(
            repo_dir=gigapose.gigapose_repo_dir,
            root_dir=data_dir,
            dataset_name=gigapose.gigapose_dataset_name,
            object_id=gigapose.gigapose_object_id,
            output_label=megapose.object_label,
            megapose_models_root=megapose.megapose_data_dir / "megapose-models",
            bsz_images=gigapose.gigapose_refiner_bsz_images,
            bsz_objects=gigapose.gigapose_refiner_bsz_objects,
            n_iterations=gigapose.gigapose_refiner_iterations,
        ),
        min_matching_score=gigapose.gigapose_min_matching_score,
        verbose=common.verbose_subprocess,
        log_info=log_info,
        log_warn=log_warn,
    )


__all__ = [
    "PoseBackendRunners",
    "create_pose_backend_runners",
]
