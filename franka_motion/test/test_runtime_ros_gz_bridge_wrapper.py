"""
Test bounded shutdown of the ros_gz_bridge process wrapper.

Responsibility: Verify escalation terminates stubborn bridge process groups without hanging.
"""

from pathlib import Path
import importlib.util
import os
import signal
import subprocess
import sys
import time


_RUNTIME_DIR = Path(__file__).resolve().parents[1] / 'scripts' / 'runtime'
_WRAPPER = _RUNTIME_DIR / 'ros_gz_bridge_wrapper.py'
_SPEC = importlib.util.spec_from_file_location('ros_gz_bridge_wrapper', _WRAPPER)
ros_gz_bridge_wrapper = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(ros_gz_bridge_wrapper)


def test_bridge_wrapper_kills_bridge_that_ignores_sigterm(tmp_path: Path):
    bridge_script = tmp_path / 'fake_parameter_bridge.py'
    bridge_pid_file = tmp_path / 'bridge.pid'
    bridge_pid_file.write_text('pending', encoding='utf-8')
    bridge_script.write_text(
        "#!/usr/bin/env python3\n"
        "import os, signal, sys, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        "pid_file = None\n"
        "for arg in sys.argv:\n"
        "    if arg.startswith('config_file:='):\n"
        "        pid_file = arg.split(':=', maxsplit=1)[1]\n"
        "if pid_file is None:\n"
        "    raise SystemExit(2)\n"
        "with open(pid_file, 'w', encoding='utf-8') as handle:\n"
        "    handle.write(str(os.getpid()))\n"
        "    handle.flush()\n"
        "while True:\n"
        "    time.sleep(1.0)\n",
        encoding='utf-8',
    )
    bridge_script.chmod(0o755)

    process: subprocess.Popen[str] | None = None
    bridge_pid: int | None = None
    try:
        process = subprocess.Popen(
            [
                sys.executable,
                str(_WRAPPER),
                '--bridge-executable',
                str(bridge_script),
                '--bridge-name',
                'ignore_shutdown_bridge_test',
                '--config-file',
                str(bridge_pid_file),
                '--term-timeout-sec',
                '0.2',
                '--kill-timeout-sec',
                '1.0',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        bridge_pid = _wait_for_child_pid(bridge_pid_file)
        started_at = time.monotonic()
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=3.0)

        assert process.returncode == 0, stdout + stderr
        assert time.monotonic() - started_at < 2.5
        assert _wait_for_process_exit(bridge_pid)
    finally:
        if process is not None:
            _terminate_process_group(process)
        if bridge_pid is not None:
            _terminate_pid(bridge_pid)


def _wait_for_child_pid(pid_file: Path) -> int:
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            return int(pid_file.read_text(encoding='utf-8'))
        except (FileNotFoundError, ValueError):
            time.sleep(0.01)
    raise AssertionError('Bridge wrapper did not start child process')


def _process_exists(pid: int) -> bool:
    try:
        stat_tail = (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', maxsplit=1)[1].strip()
        if stat_tail.split(maxsplit=1)[0] == 'Z':
            return False
    except (FileNotFoundError, IndexError, OSError):
        pass

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _wait_for_process_exit(pid: int, timeout_sec: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not _process_exists(pid):
            return True
        time.sleep(0.01)
    return not _process_exists(pid)


def _terminate_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=2.0)


def _terminate_pid(pid: int) -> None:
    if not _process_exists(pid):
        return
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
