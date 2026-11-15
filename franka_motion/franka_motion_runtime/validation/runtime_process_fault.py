"""
Fail-closed CLI for one role-marked runtime process-death transaction.

Responsibility:
- Require one exact same-user run-token/role target before injecting SIGKILL.
- Restart through the exact persisted controller identity and verify new state.
- Emit stable typed markers consumed by runtime evidence profiles.

Does not own:
- Backend child lifecycle, ROS action behavior, matrix parsing or evidence files.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from typing import Callable, Sequence

from franka_motion_runtime.validation.runtime_faultable_process import run_controller
from franka_motion_runtime.validation.runtime_process_role import (
    LinuxRoleProcessInspector,
    ProcessRoleError,
    RoleProcessState,
    RoleStateStore,
    environment_contract,
)
from franka_motion_runtime.validation.runtime_registry import ProcessIdentity


class RoleProcessControl:
    """Inject and recover one exact role process under bounded steady time."""

    def __init__(
        self,
        *,
        role: str,
        timeout_sec: float,
        clock: Callable[[], float] = time.monotonic,
        waiter: Callable[[float], object] | None = None,
    ) -> None:
        """Resolve the mandatory harness environment and state store."""
        if timeout_sec <= 0.0:
            raise ProcessRoleError("timeout_sec must be positive")
        run_token, control_directory = environment_contract(role)
        self._role = role
        self._run_token = run_token
        self._store = RoleStateStore(control_directory, role)
        self._inspector = LinuxRoleProcessInspector()
        self._timeout_sec = timeout_sec
        self._clock = clock
        self._waiter = waiter or threading.Event().wait

    def kill(self) -> RoleProcessState:
        """SIGKILL one uniquely proven role child through a Linux pidfd."""
        state = self._require_state("running")
        child = self._require_unique_child(state)
        self._signal_identity(child, signal.SIGKILL, role=self._role)
        stopped = self._wait_for(
            lambda value: (
                value.generation == state.generation
                and value.status == "stopped"
                and value.return_code == -signal.SIGKILL
            ),
            "backend did not report the injected SIGKILL",
        )
        if self._inspector.role_processes(
            run_token=self._run_token,
            role=self._role,
        ):
            raise ProcessRoleError("role process still exists after SIGKILL")
        return stopped

    def restart(self) -> RoleProcessState:
        """Request one new child generation from the exact controller PID."""
        state = self._require_state("stopped")
        if self._inspector.role_processes(
            run_token=self._run_token,
            role=self._role,
        ):
            raise ProcessRoleError("cannot restart while a role process still exists")
        self._signal_identity(state.controller, signal.SIGUSR1, role=None)
        restarted = self._wait_for(
            lambda value: (
                value.generation > state.generation
                and value.status == "running"
                and value.return_code is None
            ),
            "controller did not publish a restarted child",
        )
        self._require_unique_child(restarted)
        return restarted

    def status(self, expected_status: str) -> RoleProcessState:
        """Verify a typed stopped/running state and its live-process invariant."""
        state = self._require_state(expected_status)
        if expected_status == "running":
            self._require_unique_child(state)
        elif self._inspector.role_processes(
            run_token=self._run_token,
            role=self._role,
        ):
            raise ProcessRoleError("stopped state still has a live role process")
        return state

    def _require_state(self, expected_status: str) -> RoleProcessState:
        """Read state and prove it belongs to this exact harness run."""
        state = self._read_owned_state()
        if state.status != expected_status:
            raise ProcessRoleError(
                f"role {self._role!r} is {state.status}, expected {expected_status}"
            )
        return state

    def _read_owned_state(self) -> RoleProcessState:
        """Reject a replaced state file or stale controller identity."""
        state = self._store.read()
        if state.role != self._role or state.run_token != self._run_token:
            raise ProcessRoleError("role state does not belong to this run")
        controller = self._inspector.identity(
            state.controller.pid,
            run_token=self._run_token,
            role=None,
        )
        if controller != state.controller:
            raise ProcessRoleError("controller identity is stale or unowned")
        return state

    def _require_unique_child(self, state: RoleProcessState) -> ProcessIdentity:
        """Require state and procfs to identify exactly one identical target."""
        if state.child is None:
            raise ProcessRoleError("running state has no child identity")
        role_processes = self._inspector.role_processes(
            run_token=self._run_token,
            role=self._role,
        )
        if role_processes != (state.child,):
            raise ProcessRoleError(
                f"role target is ambiguous or stale: found {len(role_processes)} process(es)"
            )
        if state.child.process_group_id != state.child.pid:
            raise ProcessRoleError("role target does not own its process session")
        return state.child

    def _signal_identity(
        self,
        identity: ProcessIdentity,
        signal_number: int,
        *,
        role: str | None,
    ) -> None:
        """Use pidfd plus a second identity check to exclude PID reuse."""
        try:
            descriptor = os.pidfd_open(identity.pid, 0)
        except (AttributeError, OSError) as error:
            raise ProcessRoleError(f"cannot open pidfd for exact target: {error}") from error
        try:
            current = self._inspector.identity(
                identity.pid,
                run_token=self._run_token,
                role=role,
            )
            if current != identity:
                raise ProcessRoleError("process identity changed before signal")
            if role is not None:
                role_processes = self._inspector.role_processes(
                    run_token=self._run_token,
                    role=role,
                )
                if role_processes != (identity,):
                    raise ProcessRoleError(
                        "role target became ambiguous or stale before signal"
                    )
            try:
                signal.pidfd_send_signal(descriptor, signal_number)
            except (AttributeError, OSError) as error:
                raise ProcessRoleError(f"cannot signal exact pidfd target: {error}") from error
        finally:
            os.close(descriptor)

    def _wait_for(
        self,
        predicate: Callable[[RoleProcessState], bool],
        timeout_message: str,
    ) -> RoleProcessState:
        """Wait for a state transition against one monotonic deadline."""
        deadline = self._clock() + self._timeout_sec
        while True:
            state = self._read_owned_state()
            if predicate(state):
                return state
            remaining = deadline - self._clock()
            if remaining <= 0.0:
                raise ProcessRoleError(timeout_message)
            self._waiter(min(0.05, remaining))


def _parser() -> argparse.ArgumentParser:
    """Create the explicit serve/kill/restart/status command grammar."""
    parser = argparse.ArgumentParser(
        description="Control one exact role-marked runtime validation process.",
    )
    subparsers = parser.add_subparsers(dest="operation", required=True)
    serve = subparsers.add_parser("serve")
    serve.add_argument("--role", required=True)
    serve.add_argument("command", nargs=argparse.REMAINDER)
    for operation in ("kill", "restart"):
        command = subparsers.add_parser(operation)
        command.add_argument("--role", required=True)
        command.add_argument("--timeout-sec", type=float, default=10.0)
    status = subparsers.add_parser("status")
    status.add_argument("--role", required=True)
    status.add_argument("--expect", choices=("running", "stopped"), required=True)
    status.add_argument("--timeout-sec", type=float, default=10.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch one bounded role-process operation and print a typed marker."""
    arguments = _parser().parse_args(argv)
    try:
        if arguments.operation == "serve":
            command = tuple(arguments.command)
            if command[:1] == ("--",):
                command = command[1:]
            return run_controller(arguments.role, command)
        control = RoleProcessControl(
            role=arguments.role,
            timeout_sec=arguments.timeout_sec,
        )
        if arguments.operation == "kill":
            state = control.kill()
            print(
                "PROCESS_DEATH_INJECTED "
                f"role={state.role} generation={state.generation} "
                f"signal=SIGKILL exit_code={state.return_code}",
                flush=True,
            )
        elif arguments.operation == "restart":
            state = control.restart()
            print(
                "PROCESS_ROLE_RESTARTED "
                f"role={state.role} generation={state.generation}",
                flush=True,
            )
        else:
            state = control.status(arguments.expect)
            print(
                "PROCESS_ROLE_STATE "
                f"role={state.role} state={state.status} "
                f"generation={state.generation} exit_code={state.return_code}",
                flush=True,
            )
        return 0
    except (OSError, ProcessRoleError, ValueError) as error:
        print(f"PROCESS_ROLE_ERROR: {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
