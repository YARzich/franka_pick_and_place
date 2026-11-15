"""
Focused fakes and runner factories shared by pose-backend tests.

Responsibility: Build deterministic MegaPose/GigaPose runners without ROS,
GPU models, or real subprocesses.
"""

import json
from pathlib import Path
import sys
from types import SimpleNamespace

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

K_VALUES = [100.0, 0.0, 50.0, 0.0, 110.0, 60.0, 0.0, 0.0, 1.0]
POSE = {'label': 'cube_5cm', 'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]]}


class FakeWorkerClient:
    def __init__(
        self,
        responses: list[dict] | None = None,
        exc: Exception | None = None,
        *,
        persistent: bool = True,
    ) -> None:
        self.responses = list(responses or [])
        self.exc = exc
        self.persistent = persistent
        self.requests: list[dict] = []
        self.stopped = False

    def request(self, request: dict) -> dict:
        self.requests.append(request)
        if self.exc is not None:
            raise self.exc
        return self.responses.pop(0)

    def stop(self) -> None:
        self.stopped = True


class FakeRuntimeExample:
    def __init__(self, output_pose_paths: list[Path]) -> None:
        self.output_pose_paths = output_pose_paths
        self.prepare_calls: list[tuple] = []

    def prepare(self, rgb, depth, use_depth, camera_k, bbox) -> None:
        self.prepare_calls.append((rgb, depth, use_depth, camera_k, bbox))


class FakeRuntimeInput:
    def __init__(self, image_path: Path, mask_path: Path) -> None:
        self.image_path = image_path
        self.mask_path = mask_path
        self.prepare_calls: list[tuple] = []

    def prepare(self, rgb, mask):
        self.prepare_calls.append((rgb, mask))
        return self.image_path, self.mask_path if mask is not None else None


class FakeSubprocessRun:
    def __init__(self, returncode: int = 0, stdout: str = '') -> None:
        self.returncode = returncode
        self.stdout = stdout
        self.calls: list[tuple] = []

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return SimpleNamespace(returncode=self.returncode, stdout=self.stdout, stderr='cli error')


class LogCapture:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


def _write_output_pose(path: Path, pose: dict = POSE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps([pose]))


def _megapose_runner(
    tmp_path: Path,
    *,
    worker: FakeWorkerClient,
    use_worker: bool = True,
    verbose: bool = True,
    output_pose: dict | None = None,
) -> tuple[MegaPoseBackendRunner, FakeSubprocessRun, LogCapture, FakeRuntimeExample]:
    output_path = tmp_path / 'outputs' / 'object_data.json'
    if output_pose is not None:
        _write_output_pose(output_path, output_pose)
    runtime_example = FakeRuntimeExample([output_path])
    subprocess_run = FakeSubprocessRun(stdout='cli ok')
    logs = LogCapture()
    runner = MegaPoseBackendRunner(
        runtime_example=runtime_example,
        worker_client=worker,
        worker_config=MegaPoseWorkerRequestConfig(
            example_name='franka_motion_cube_5cm',
            example_dir=tmp_path / 'example',
            object_label='cube_5cm',
            bsz_images=64,
            bsz_objects=8,
            renderer_workers=1,
        ),
        cli_config=MegaPoseCliConfig(
            python_path=sys.executable,
            example_name='franka_motion_cube_5cm',
            output_pose_paths=runtime_example.output_pose_paths,
            object_label='cube_5cm',
            process_timeout_sec=10.0,
            env_factory=lambda: {'TEST_ENV': '1'},
        ),
        use_worker=use_worker,
        verbose=verbose,
        log_info=logs.info,
        log_warn=logs.warn,
        log_error=logs.error,
        subprocess_run=subprocess_run,
    )
    return runner, subprocess_run, logs, runtime_example


def _gigapose_runner(
    tmp_path: Path,
    *,
    worker: FakeWorkerClient,
    refiner: FakeWorkerClient,
    verbose: bool = True,
    min_matching_score: float = 0.0,
) -> tuple[GigaPosePipelineRunner, LogCapture, FakeRuntimeInput]:
    logs = LogCapture()
    runtime_input = FakeRuntimeInput(tmp_path / 'image_rgb.png', tmp_path / 'image_mask.png')
    runner = GigaPosePipelineRunner(
        runtime_input=runtime_input,
        worker_client=worker,
        refiner_worker_client=refiner,
        worker_config=GigaPoseWorkerRequestConfig(
            repo_dir=Path('/repo'),
            dinov2_repo_dir=None,
            root_dir=Path('/data'),
            dataset_name='franka_cube',
            object_id=1,
            checkpoint_path=Path('/checkpoint.ckpt'),
            template_count=162,
            max_num_dets_per_forward=4,
            translation_scale=0.001,
            num_hypotheses=5,
        ),
        refiner_config=GigaPoseRefinerRequestConfig(
            repo_dir=Path('/repo'),
            root_dir=Path('/data'),
            dataset_name='franka_cube',
            object_id=1,
            output_label='cube_5cm',
            megapose_models_root=Path('/megapose-models'),
            bsz_images=512,
            bsz_objects=8,
            n_iterations=5,
        ),
        min_matching_score=min_matching_score,
        verbose=verbose,
        log_info=logs.info,
        log_warn=logs.warn,
    )
    return runner, logs, runtime_input
