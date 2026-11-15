"""
Linux process helpers shared by subprocess-ownership tests.

Responsibility: Create disposable worker process trees and verify deterministic
process-group termination without duplicating /proc polling logic.
"""

import os
from pathlib import Path
import signal
import time

_GRANDCHILD_CODE = (
    "import os, signal, sys, time\n"
    "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
    "with open(sys.argv[1], 'w', encoding='utf-8') as pid_file:\n"
    "    pid_file.write(str(os.getpid()))\n"
    "    pid_file.flush()\n"
    "while True:\n"
    "    time.sleep(1.0)\n"
)


def _env() -> dict[str, str]:
    return dict(os.environ)


def _wait_until(predicate, timeout_sec: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def _write_grandchild_spawning_worker(
    worker: Path,
    grandchild_pid_file: Path,
    *,
    persistent: bool,
) -> None:
    spawn_line = (
        "subprocess.Popen("
        f"[sys.executable, '-c', {_GRANDCHILD_CODE!r}, {str(grandchild_pid_file)!r}]"
        ")\n"
    )
    if persistent:
        worker.write_text(
            "import json, subprocess, sys, time\n"
            "for line in sys.stdin:\n"
            "    request = json.loads(line)\n"
            "    if request.get('command') == 'shutdown':\n"
            "        break\n"
            f"    {spawn_line}"
            "    print('started', flush=True)\n"
            "    time.sleep(30)\n"
            "    print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
        )
        return

    worker.write_text(
        "import json, subprocess, sys, time\n"
        "request = json.loads(sys.stdin.readline())\n"
        f"{spawn_line}"
        "time.sleep(30)\n"
        "print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
    )


def _write_graceful_grandchild_worker(worker: Path, grandchild_pid_file: Path) -> None:
    spawn_line = (
        "subprocess.Popen("
        f"[sys.executable, '-c', {_GRANDCHILD_CODE!r}, {str(grandchild_pid_file)!r}]"
        ")\n"
    )
    worker.write_text(
        "import json, subprocess, sys\n"
        "for line in sys.stdin:\n"
        "    request = json.loads(line)\n"
        "    if request.get('command') == 'shutdown':\n"
        "        break\n"
        f"    {spawn_line}"
        "    print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
    )


def _write_exiting_grandchild_worker(worker: Path, grandchild_pid_file: Path) -> None:
    spawn_line = (
        "subprocess.Popen("
        f"[sys.executable, '-c', {_GRANDCHILD_CODE!r}, {str(grandchild_pid_file)!r}]"
        ")\n"
    )
    worker.write_text(
        "import json, os, subprocess, sys, time\n"
        "json.loads(sys.stdin.readline())\n"
        f"{spawn_line}"
        "deadline = time.monotonic() + 1.0\n"
        f"while time.monotonic() < deadline and "
        f"not os.path.exists({str(grandchild_pid_file)!r}):\n"
        "    time.sleep(0.01)\n"
        "sys.exit(7)\n"
    )


def _write_idle_grandchild_worker(worker: Path, grandchild_pid_file: Path) -> None:
    spawn_line = (
        "subprocess.Popen("
        f"[sys.executable, '-c', {_GRANDCHILD_CODE!r}, {str(grandchild_pid_file)!r}]"
        ")\n"
    )
    worker.write_text(
        "import json, subprocess, sys, time\n"
        "request = json.loads(sys.stdin.readline())\n"
        f"{spawn_line}"
        "print(json.dumps({'id': request['id'], 'ok': True}), flush=True)\n"
        "while True:\n"
        "    time.sleep(1.0)\n"
    )


def _wait_for_pid_file(pid_file: Path, timeout_sec: float = 1.0) -> int:
    def pid_file_is_ready() -> bool:
        try:
            return pid_file.read_text().strip().isdigit()
        except OSError:
            return False

    assert _wait_until(pid_file_is_ready, timeout_sec=timeout_sec)
    return int(pid_file.read_text().strip())


def _process_is_running(pid: int) -> bool:
    try:
        stat_path = Path('/proc') / str(pid) / 'stat'
        stat_tail = stat_path.read_text().rsplit(')', maxsplit=1)[1].strip()
        if stat_tail.split(maxsplit=1)[0] == 'Z':
            return False
    except (FileNotFoundError, IndexError, OSError):
        pass

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _wait_for_process_exit(pid: int, timeout_sec: float = 3.0) -> bool:
    return _wait_until(lambda: not _process_is_running(pid), timeout_sec=timeout_sec)


def _kill_leftover_pid(pid: int) -> None:
    if not _process_is_running(pid):
        return
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
