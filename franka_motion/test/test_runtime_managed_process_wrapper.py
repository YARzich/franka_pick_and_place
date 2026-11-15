"""
Test process-group cleanup performed by the managed-process wrapper.

Responsibility: Ensure stubborn children and surviving helpers are killed within bounded time.
"""

from pathlib import Path
import os
import signal
import subprocess
import sys
import time


_WRAPPER = Path(__file__).resolve().parents[1] / 'scripts' / 'runtime' / 'managed_process_wrapper.py'


def test_managed_process_wrapper_kills_child_that_ignores_shutdown_signal(tmp_path: Path):
    child_script = tmp_path / 'ignore_shutdown.py'
    child_pid_file = tmp_path / 'child.pid'
    child_script.write_text(
        "import os, signal, sys, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        "with open(sys.argv[1], 'w', encoding='utf-8') as pid_file:\n"
        "    pid_file.write(str(os.getpid()))\n"
        "    pid_file.flush()\n"
        "while True:\n"
        "    time.sleep(1.0)\n"
    )
    process: subprocess.Popen[str] | None = None
    child_pid: int | None = None
    try:
        process = subprocess.Popen(
            [
                sys.executable,
                str(_WRAPPER),
                '--child-name',
                'ignore_shutdown_test',
                '--shutdown-signal',
                'term',
                '--shutdown-timeout-sec',
                '0.2',
                '--kill-timeout-sec',
                '1.0',
                '--',
                sys.executable,
                str(child_script),
                str(child_pid_file),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        child_pid = _wait_for_child_pid(child_pid_file)
        started_at = time.monotonic()
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=3.0)

        assert process.returncode == 0, stdout + stderr
        assert time.monotonic() - started_at < 2.5
        assert _wait_for_process_exit(child_pid)
    finally:
        if process is not None:
            _terminate_process_group(process)
        if child_pid is not None:
            _terminate_pid(child_pid)


def test_managed_process_wrapper_kills_helper_after_direct_child_exits(tmp_path: Path):
    child_script = tmp_path / 'spawn_helper.py'
    helper_script = tmp_path / 'ignore_shutdown_helper.py'
    helper_pid_file = tmp_path / 'helper.pid'
    helper_script.write_text(
        "import os, signal, sys, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        "with open(sys.argv[1], 'w', encoding='utf-8') as pid_file:\n"
        "    pid_file.write(str(os.getpid()))\n"
        "    pid_file.flush()\n"
        "while True:\n"
        "    time.sleep(1.0)\n"
    )
    child_script.write_text(
        "import os, subprocess, sys, time\n"
        "subprocess.Popen([sys.executable, sys.argv[1], sys.argv[2]])\n"
        "deadline = time.monotonic() + 1.0\n"
        "while time.monotonic() < deadline and not os.path.exists(sys.argv[2]):\n"
        "    time.sleep(0.01)\n"
    )
    process: subprocess.Popen[str] | None = None
    helper_pid: int | None = None
    try:
        process = subprocess.Popen(
            [
                sys.executable,
                str(_WRAPPER),
                '--child-name',
                'helper_shutdown_test',
                '--shutdown-signal',
                'term',
                '--shutdown-timeout-sec',
                '0.2',
                '--kill-timeout-sec',
                '1.0',
                '--',
                sys.executable,
                str(child_script),
                str(helper_script),
                str(helper_pid_file),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        helper_pid = _wait_for_child_pid(helper_pid_file)
        started_at = time.monotonic()
        stdout, stderr = process.communicate(timeout=3.0)

        assert process.returncode == 0, stdout + stderr
        assert time.monotonic() - started_at < 2.5
        assert _wait_for_process_exit(helper_pid)
    finally:
        if process is not None:
            _terminate_process_group(process)
        if helper_pid is not None:
            _terminate_pid(helper_pid)


def _wait_for_child_pid(pid_file: Path) -> int:
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if pid_file.exists():
            return int(pid_file.read_text(encoding='utf-8'))
        time.sleep(0.01)
    raise AssertionError('Managed wrapper did not start child process')


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
