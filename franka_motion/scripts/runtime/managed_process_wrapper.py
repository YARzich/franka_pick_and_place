#!/usr/bin/env python3
"""
Managed subprocess wrapper for launch-controlled shutdown.

Responsibility:
- Start one child command in a separate process session.
- Convert wrapper SIGINT/SIGTERM into the configured child shutdown signal.
- Force SIGKILL when the child ignores the configured shutdown signal.
- Clean up helper descendants left in the child process group after direct child exit.
- Return cleanly to ROS launch when shutdown was requested.

Does not own:
- ROS node behavior inside the child process.
- Child command construction beyond passed CLI arguments.
- Gazebo/Isaac-specific cleanup policy.
"""

from __future__ import annotations

import argparse
import signal
import sys

try:
    from franka_motion_runtime.processes import process_group_runner as runner_module
except ModuleNotFoundError as error:
    if error.name not in {
        "franka_motion_runtime",
        "franka_motion_runtime.processes",
    }:
        raise
    # Isaac launch may replace PYTHONPATH; the installed wrapper carries this
    # private helper beside itself so process ownership remains self-contained.
    import process_group_runner as runner_module

ProcessGroupRunner = runner_module.ProcessGroupRunner
ProcessGroupRunnerConfig = runner_module.ProcessGroupRunnerConfig


class ManagedProcessWrapper:
    """Builds generic wrapper config and delegates process lifecycle."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Store validated command-line arguments for runner construction."""
        self._args = args

    def _get_shutdown_signal(self) -> signal.Signals:
        signal_map = {
            "int": signal.SIGINT,
            "term": signal.SIGTERM,
            "kill": signal.SIGKILL,
        }
        return signal_map[self._args.shutdown_signal]

    def _build_runner_config(self) -> ProcessGroupRunnerConfig:
        return ProcessGroupRunnerConfig(
            command=self._args.command,
            child_name=self._args.child_name,
            shutdown_signal=self._get_shutdown_signal(),
            shutdown_timeout_sec=self._args.shutdown_timeout_sec,
            kill_timeout_sec=self._args.kill_timeout_sec,
            cleanup_on_child_exit=True,
        )

    def run(self) -> int:
        """Run the configured child under bounded process-group ownership."""
        return ProcessGroupRunner(self._build_runner_config()).run()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Wrapper для управляемого завершения дочернего процесса."
    )
    parser.add_argument(
        "--child-name",
        default="managed_process",
        help="Логическое имя дочернего процесса для отладки.",
    )
    parser.add_argument(
        "--shutdown-signal",
        choices=("int", "term", "kill"),
        default="term",
        help=(
            "Какой сигнал отправлять дочернему процессу "
            "при shutdown wrapper-а."
        ),
    )
    parser.add_argument(
        "--shutdown-timeout-sec",
        type=float,
        default=3.0,
        help="Сколько ждать после shutdown signal перед принудительным SIGKILL.",
    )
    parser.add_argument(
        "--kill-timeout-sec",
        type=float,
        default=2.0,
        help="Сколько ждать после SIGKILL перед выходом wrapper-а.",
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    parsed_args = parser.parse_args()
    if parsed_args.command and parsed_args.command[0] == "--":
        parsed_args.command = parsed_args.command[1:]
    if not parsed_args.command:
        parser.error("Не передана команда дочернего процесса")
    if parsed_args.shutdown_timeout_sec < 0.0:
        parser.error("--shutdown-timeout-sec должен быть >= 0")
    if parsed_args.kill_timeout_sec < 0.0:
        parser.error("--kill-timeout-sec должен быть >= 0")
    return parsed_args


def main() -> None:
    """Parse CLI arguments, run the child and return its terminal status."""
    args = _parse_args()
    wrapper = ManagedProcessWrapper(args)
    try:
        return_code = wrapper.run()
    except KeyboardInterrupt:
        return_code = 0
    sys.exit(return_code)


if __name__ == "__main__":
    main()
