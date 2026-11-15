"""
Stable CLI facade for runtime execution and interrupted-run cleanup.

Responsibility:
- Parse CLI arguments, install signal policy and call runtime services.
- Keep the installed entrypoint small and stable.

Does not own:
- Stage semantics, process ownership, registry persistence or backend commands.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
from typing import Mapping, Sequence
import uuid

from franka_motion_runtime.validation.runtime_evidence import HarnessOutcome
from franka_motion_runtime.validation.runtime_orchestrator import RuntimeHarness
from franka_motion_runtime.validation.runtime_process import ProcessError
from franka_motion_runtime.validation.runtime_service import (
    cleanup_profile,
    run_profile,
)

__all__ = [
    "HarnessOutcome",
    "RuntimeHarness",
    "cleanup_profile",
    "main",
    "run_profile",
]


class HarnessInterrupted(ProcessError):
    """Convert SIGINT/SIGTERM into an exception that executes cleanup."""


def _install_signal_handlers() -> dict[int, object]:
    """Install exception-raising handlers so finally cleanup runs."""
    previous_handlers = {}

    def raise_interrupted(signal_number, frame) -> None:
        """Raise through normal control flow so service cleanup executes."""
        del frame
        raise HarnessInterrupted(
            f"runtime harness received signal {signal_number}"
        )

    for signal_number in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signal_number] = signal.getsignal(signal_number)
        signal.signal(signal_number, raise_interrupted)
    return previous_handlers


def _restore_signal_handlers(previous_handlers: Mapping[int, object]) -> None:
    """Restore caller signal policy after command completion."""
    for signal_number, handler in previous_handlers.items():
        signal.signal(signal_number, handler)


def _create_argument_parser() -> argparse.ArgumentParser:
    """Define the small CLI shared by all runtime workflows."""
    parser = argparse.ArgumentParser(
        description="Run bounded Franka runtime validation profiles.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command_name in ("run", "cleanup"):
        command_parser = subparsers.add_parser(command_name)
        command_parser.add_argument("--matrix", type=Path, required=True)
        command_parser.add_argument("--profile", required=True)
        command_parser.add_argument(
            "--artifact-dir",
            type=Path,
            required=True,
        )
        command_parser.add_argument("--state-file", type=Path, required=True)
        command_parser.add_argument("--run-token")
    return parser


def _execute(arguments) -> HarnessOutcome:
    """Route parsed CLI arguments without duplicating infrastructure wiring."""
    run_token = arguments.run_token or uuid.uuid4().hex
    if arguments.command == "run":
        return run_profile(
            matrix_path=arguments.matrix,
            profile_id=arguments.profile,
            artifact_directory=arguments.artifact_dir,
            state_path=arguments.state_file,
            run_token=run_token,
            environment=os.environ,
        )
    return cleanup_profile(
        matrix_path=arguments.matrix,
        profile_id=arguments.profile,
        artifact_directory=arguments.artifact_dir,
        state_path=arguments.state_file,
        requested_run_token=arguments.run_token,
        environment=os.environ,
    )


def main(argv: Sequence[str] | None = None) -> int:
    """Execute one CLI command and return its evidence-aware exit status."""
    arguments = _create_argument_parser().parse_args(argv)
    previous_handlers = _install_signal_handlers()
    try:
        outcome = _execute(arguments)
    finally:
        _restore_signal_handlers(previous_handlers)
    print(
        json.dumps(
            {
                "result_path": str(outcome.result_path),
                "evidence_status": outcome.evidence_status,
                "primary_error": outcome.primary_error,
                "cleanup_errors": outcome.cleanup_errors,
            },
            sort_keys=True,
        )
    )
    return outcome.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
