"""
Provide deterministic fakes and factories for pose lifecycle tests.

Responsibility: Isolate pose runner and input-coordinator tests from ROS
executors, GPU inference backends, and asynchronous worker timing.
"""

from types import SimpleNamespace

import numpy as np
from std_msgs.msg import Header

from manipulation_perception.common.header_sync import header_key
from manipulation_perception.pose.pose_inference_gate import PoseInferenceGate
from manipulation_perception.pose.pose_inference_runner import MegaPoseInferenceRunner
from manipulation_perception.pose.pose_input_coordinator import PoseInputCoordinator
from manipulation_perception.pose.pose_pending_frames import PendingPoseFrames


POSE = {'label': 'cube_5cm', 'TWO': [[0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]]}


class FakeBackend:
    def __init__(self) -> None:
        self.lifecycle = None
        self.calls = 0
        self.result = POSE
        self.error = None
        self.stopped = False

    def run_with_runtime_example(self, *_args):
        self.calls += 1
        if self.lifecycle is not None:
            self.lifecycle.request_shutdown()
        if self.error is not None:
            raise self.error
        return self.result

    def stop(self) -> None:
        self.stopped = True


class FakeGigaPosePipeline:
    def __init__(self) -> None:
        self.calls = 0
        self.stopped = False

    def run_refined(self, *_args):
        self.calls += 1
        return POSE

    def stop(self) -> None:
        self.stopped = True


class FakeLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def warn(self, message: str, **_kwargs) -> None:
        self.warnings.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeInputBuffer:
    def __init__(self) -> None:
        self.ready = False
        self.empty_bbox = False
        self.clear_calls = 0

    @staticmethod
    def message_key(message):
        return header_key(getattr(message, 'header', None))

    def context_for_image(self, _image_message, **_kwargs):
        if not self.ready:
            return None
        return SimpleNamespace(camera_info='camera', bbox=(1, 1, 2, 2))

    def has_empty_bbox_response_for_image(self, _image_message) -> bool:
        return self.empty_bbox

    @staticmethod
    def decode_snapshot(_image_message, context):
        return context

    def clear(self) -> None:
        self.clear_calls += 1
        self.ready = False
        self.empty_bbox = False


class KeyedFakeInputBuffer(FakeInputBuffer):
    def __init__(self) -> None:
        super().__init__()
        self.ready_keys = set()

    def update_bbox_detections(self, message) -> None:
        key = self.message_key(message)
        if key is not None:
            self.ready_keys.add(key)

    def context_for_image(self, image_message, **_kwargs):
        if self.message_key(image_message) not in self.ready_keys:
            return None
        return SimpleNamespace(camera_info='camera', bbox=(1, 1, 2, 2))


class FakeCoordinatorRunner:
    def __init__(self) -> None:
        self.gate = PoseInferenceGate(run_every_n_frames=1, min_interval_sec=0.0)
        self.generation = 0
        self.started = []
        self.invalidated = []

    @staticmethod
    def shutdown_requested() -> bool:
        return False

    def accepts_frame(self) -> bool:
        return self.gate.accepts_frame()

    def is_running(self) -> bool:
        return self.gate.is_running()

    def try_start(self, now_monotonic: float) -> bool:
        return self.gate.try_start(now_monotonic)

    def mark_start_failed(self) -> None:
        self.gate.mark_done()

    def current_generation(self) -> int:
        return self.generation

    def reset_for_clock_jump(self) -> None:
        self.generation += 1

    @staticmethod
    def requires_depth_before_inference() -> bool:
        return False

    def start(self, snapshot, generation: int) -> None:
        self.started.append((snapshot, generation))

    def publish_invalidation(self, image_header, generation: int) -> None:
        self.invalidated.append((image_header, generation))


def make_header(sec=1, nanosec=0, frame_id='table_stereo_left_optical_frame'):
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def make_image_message(header=None):
    return SimpleNamespace(header=header if header is not None else make_header())


def make_pending_frames(input_buffer, *frames, timeout_sec: float = 1.0):
    pending = PendingPoseFrames(input_lookup=input_buffer, timeout_sec=timeout_sec)
    for image_message, received_at in frames:
        key = header_key(image_message.header)
        assert key is not None
        pending.add(key, image_message, received_at, 0)
    return pending


def make_coordinator(input_buffer, *frames, timeout_sec: float = 1.0):
    coordinator = object.__new__(PoseInputCoordinator)
    runner = FakeCoordinatorRunner()
    logger = FakeLogger()
    coordinator._runner = runner
    coordinator._logger = logger
    # This fixture bypasses __init__ to inject a deterministic input buffer.
    # Keep observer callbacks explicit as part of the coordinator contract.
    coordinator._on_input = lambda: None
    coordinator._on_drop_or_skip = lambda _count: None
    coordinator._on_error = lambda: None
    coordinator._input_buffer = input_buffer
    coordinator._pending_frames = make_pending_frames(
        input_buffer,
        *frames,
        timeout_sec=timeout_sec,
    )
    return coordinator, runner, logger


def make_actual_runner(*, backend=None, logger=None):
    backend = backend or FakeBackend()
    gigapose = FakeGigaPosePipeline()
    logger = logger or FakeLogger()
    publisher = FakePublisher()
    parameters = SimpleNamespace(
        common=SimpleNamespace(
            coarse_estimator='megapose',
            depth_mode='off',
            process_timeout_sec=1.0,
            output_frame_id='table_stereo_left_optical_frame',
            score=0.9,
            run_every_n_frames=1,
            min_inference_interval_sec=0.0,
        ),
        megapose=SimpleNamespace(
            megapose_model='megapose-model',
            object_label='cube_5cm',
        ),
    )
    runner = MegaPoseInferenceRunner(
        parameters=parameters,
        megapose_backend=backend,
        gigapose_pipeline=gigapose,
        publish=publisher.publish,
        object_size_m=(0.05, 0.05, 0.05),
        logger=logger,
    )
    return runner, backend, gigapose, logger, publisher


def run_backend_once(runner, image_header) -> None:
    assert runner.try_start(10.0)
    runner._run_worker(
        np.zeros((2, 2, 3), dtype=np.uint8),
        None,
        None,
        False,
        'megapose-model',
        SimpleNamespace(k=[1.0] * 9),
        image_header,
        (0, 0, 1, 1),
        runner.current_generation(),
    )
