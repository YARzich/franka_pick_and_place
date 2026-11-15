"""
Test end-to-end cleanup of the Gazebo and MoveIt compatibility scenario.

Responsibility: Ensure repeated Ctrl-C shutdown leaves no owned simulator or ROS processes.
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


_LAUNCH_TIMEOUT_SEC = 75.0
_SHUTDOWN_TIMEOUT_SEC = 25.0
_POST_SHUTDOWN_TIMEOUT_SEC = 10.0
_CLOCK_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
_RESIDUAL_PATTERNS = (
    'ros2 launch franka_motion',
    'ros_gz_bridge_wrapper.py',
    '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge',
    'managed_process_wrapper.py',
    '/opt/ros/jazzy/lib/moveit_ros_move_group/move_group',
    '/opt/ros/jazzy/lib/controller_manager/ros2_control_node',
    '/opt/ros/jazzy/lib/controller_manager/spawner',
    '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher',
    '/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
    'panda_gripper_driver',
    'component_container_mt',
    'image_bridge',
    'stereo_sgbm_pointcloud_node.py',
    'disparity_frame_fix_node.py',
    'foundationpose_.*_node.py',
    'megapose_detection_node.py',
    'yolo_visualizer_node.py',
    'gz sim',
    'gzserver',
    'gzclient',
)
_SHUTDOWN_ERROR_MARKERS = (
    'segmentation fault',
    'exit code -11',
    'address not mapped',
)


class _LaunchRun:
    def __init__(self, command: list[str]):
        self._command = command
        self._lines: list[str] = []
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None

    @property
    def output(self) -> str:
        return ''.join(self._lines)

    @property
    def tail(self) -> str:
        return ''.join(self._lines[-120:])

    def start(self) -> None:
        self._process = subprocess.Popen(
            self._command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding='utf-8',
            errors='replace',
            bufsize=1,
            env={**os.environ, 'RCUTILS_COLORIZED_OUTPUT': '0'},
            start_new_session=True,
        )
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    def assert_running(self) -> None:
        process = self._require_process()
        if process.poll() is not None:
            raise AssertionError(
                f'Launch exited early with code {process.returncode}\n'
                f'Output tail:\n{self.tail}'
            )

    def stop_with_sigint(self) -> int:
        process = self._require_process()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)

        try:
            returncode = process.wait(timeout=_SHUTDOWN_TIMEOUT_SEC)
        except subprocess.TimeoutExpired as exc:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                returncode = process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                returncode = process.wait(timeout=5.0)
            raise AssertionError(
                'Launch did not stop after SIGINT within '
                f'{_SHUTDOWN_TIMEOUT_SEC}s\nOutput tail:\n{self.tail}'
            ) from exc

        if self._reader is not None:
            self._reader.join(timeout=2.0)
        return returncode

    def terminate_for_cleanup(self) -> None:
        process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)

    def _read_output(self) -> None:
        process = self._require_process()
        assert process.stdout is not None
        for line in process.stdout:
            self._lines.append(line)

    def _require_process(self) -> subprocess.Popen[str]:
        if self._process is None:
            raise RuntimeError('Launch process has not been started')
        return self._process


def test_gazebo_moveit_ctrl_c_shutdown_can_run_twice():
    launch_command = [
        'ros2',
        'launch',
        'franka_motion',
        'franka_gazebo_moveit.launch.py',
        'use_sim_time:=true',
        'headless:=true',
    ]

    _assert_no_residual_processes()
    for _ in range(2):
        _run_launch_until_ready_then_shutdown(launch_command)
        _wait_for_no_residual_processes()


def _run_launch_until_ready_then_shutdown(command: list[str]) -> None:
    launch_run = _LaunchRun(command)
    launch_run.start()
    try:
        _wait_for_moveit_runtime_ready(launch_run)
        returncode = launch_run.stop_with_sigint()
        _assert_shutdown_output_is_clean(launch_run.output, returncode)
    finally:
        launch_run.terminate_for_cleanup()


def _wait_for_moveit_runtime_ready(launch_run: _LaunchRun) -> None:
    deadline = time.monotonic() + _LAUNCH_TIMEOUT_SEC
    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node('test_shutdown_cleanup', context=context)
    executor.add_node(node)
    try:
        _wait_for_message(executor, node, Clock, '/clock', _remaining(deadline), _CLOCK_QOS)
        launch_run.assert_running()
        _wait_for_message(executor, node, JointState, '/joint_states', _remaining(deadline), 10)
        launch_run.assert_running()
        _wait_for_service(executor, node, GetParameters, '/move_group/get_parameters', _remaining(deadline))
        launch_run.assert_running()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


def _wait_for_message(executor, node, msg_type, topic: str, timeout_sec: float, qos_profile) -> None:
    event = threading.Event()

    def _callback(_msg):
        event.set()

    subscription = node.create_subscription(msg_type, topic, _callback, qos_profile)
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok(context=node.context):
            executor.spin_once(timeout_sec=0.1)
            if event.is_set():
                return
    finally:
        node.destroy_subscription(subscription)

    raise AssertionError(f'{topic} did not publish within {timeout_sec:.1f}s')


def _wait_for_service(executor, node, service_type, service_name: str, timeout_sec: float) -> None:
    client = node.create_client(service_type, service_name)
    try:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok(context=node.context):
            if client.wait_for_service(timeout_sec=0.2):
                return
    finally:
        node.destroy_client(client)

    raise AssertionError(f'{service_name} was not available within {timeout_sec:.1f}s')


def _assert_shutdown_output_is_clean(output: str, returncode: int) -> None:
    lowered_output = output.lower()
    for marker in _SHUTDOWN_ERROR_MARKERS:
        assert marker not in lowered_output, (
            f'Launch shutdown output contains {marker!r}\n'
            f'Output tail:\n{_tail(output)}'
        )
    assert returncode in (0, 130, -signal.SIGINT), (
        f'Unexpected launch return code after SIGINT: {returncode}\n'
        f'Output tail:\n{_tail(output)}'
    )


def _assert_no_residual_processes() -> None:
    residuals = _find_residual_processes()
    assert not residuals, (
        'ROS/Gazebo processes are already running before shutdown test:\n'
        + '\n'.join(residuals)
    )


def _wait_for_no_residual_processes() -> None:
    deadline = time.monotonic() + _POST_SHUTDOWN_TIMEOUT_SEC
    residuals = _find_residual_processes()
    while residuals and time.monotonic() < deadline:
        time.sleep(0.5)
        residuals = _find_residual_processes()

    assert not residuals, (
        'ROS/Gazebo processes remained after shutdown:\n'
        + '\n'.join(residuals)
    )


def _find_residual_processes() -> list[str]:
    residuals: list[str] = []
    own_pid = os.getpid()
    for pattern in _RESIDUAL_PATTERNS:
        result = subprocess.run(
            ['pgrep', '-af', pattern],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        for line in result.stdout.splitlines():
            if not line:
                continue
            pid_text = line.split(maxsplit=1)[0]
            if pid_text.isdigit() and int(pid_text) == own_pid:
                continue
            residuals.append(f'{pattern}: {line}')
    return residuals


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise AssertionError('Timed out waiting for launch runtime readiness')
    return remaining


def _tail(output: str) -> str:
    return ''.join(output.splitlines(keepends=True)[-120:])
