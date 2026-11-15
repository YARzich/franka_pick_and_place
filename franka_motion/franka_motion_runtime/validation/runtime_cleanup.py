"""
Idempotent signal escalation for proven owned process groups.

Responsibility:
- Signal only groups whose same-user processes carry an exact run token.
- Wait for absence after TERM and KILL and report every independent error.

Does not own:
- Process creation, direct-child joins, registry writes or stage semantics.
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import threading
import time
from typing import Callable

from franka_motion_runtime.validation.runtime_deadline import CleanupReport
from franka_motion_runtime.validation.runtime_registry import (
    LinuxProcessInspector,
    ProcessInspector,
    ProcessRegistry,
)


class OwnedGroupCleaner:
    """Terminate and then kill only groups carrying an exact run token."""

    def __init__(
        self,
        inspector: ProcessInspector | None = None,
        *,
        signal_group: Callable[[int, int], None] = os.killpg,
        waiter: Callable[[float], object] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Inject ownership inspection, signaling and steady-time waits."""
        self._inspector = inspector or LinuxProcessInspector()
        self._signal_group = signal_group
        self._waiter = waiter or threading.Event().wait
        self._clock = clock

    def cleanup(
        self,
        run_token: str,
        *,
        terminate_grace_sec: float = 10.0,
        kill_grace_sec: float = 5.0,
    ) -> CleanupReport:
        """Use TERM, bounded absence checks and KILL without name matching."""
        errors = []
        try:
            terminated_groups = self._signal_owned_groups(
                run_token,
                signal.SIGTERM,
                errors,
            )
            remaining_groups = self._wait_for_absence(
                run_token,
                terminate_grace_sec,
            )
        except (OSError, ValueError) as error:
            errors.append(f"cannot inspect owned process groups: {error}")
            return CleanupReport(run_token, (), (), tuple(errors))

        killed_groups: tuple[int, ...] = ()
        if remaining_groups:
            try:
                killed_groups = self._signal_owned_groups(
                    run_token,
                    signal.SIGKILL,
                    errors,
                )
                remaining_groups = self._wait_for_absence(
                    run_token,
                    kill_grace_sec,
                )
            except (OSError, ValueError) as error:
                errors.append(f"cannot verify killed process groups: {error}")
        if remaining_groups:
            errors.append(
                "owned process groups survived SIGKILL: "
                + ", ".join(str(group) for group in sorted(remaining_groups))
            )
        return CleanupReport(
            run_token=run_token,
            terminated_groups=terminated_groups,
            killed_groups=killed_groups,
            errors=tuple(errors),
        )

    def _signal_owned_groups(
        self,
        run_token: str,
        signal_number: int,
        errors: list[str],
    ) -> tuple[int, ...]:
        """Signal groups after re-verifying exact process identities."""
        candidates = {}
        for process in self._inspector.owned_processes(run_token):
            if process.process_group_id > 1:
                candidates.setdefault(
                    process.process_group_id,
                    set(),
                ).add(process)
        signaled = []
        for process_group_id in sorted(candidates, reverse=True):
            current_identities = set(
                self._inspector.owned_processes(run_token)
            )
            if not candidates[process_group_id] & current_identities:
                continue
            try:
                self._signal_group(process_group_id, signal_number)
            except ProcessLookupError:
                continue
            except (OSError, PermissionError) as error:
                errors.append(
                    f"cannot signal process group {process_group_id}: {error}"
                )
            else:
                signaled.append(process_group_id)
        return tuple(signaled)

    def _wait_for_absence(
        self,
        run_token: str,
        timeout_sec: float,
    ) -> set[int]:
        """Wait to a monotonic deadline and return groups still owned."""
        deadline = self._clock() + timeout_sec
        while True:
            remaining_groups = {
                process.process_group_id
                for process in self._inspector.owned_processes(run_token)
            }
            if not remaining_groups:
                return set()
            remaining_time = deadline - self._clock()
            if remaining_time <= 0.0:
                return remaining_groups
            self._waiter(min(0.05, remaining_time))


def cleanup_registered_run(
    state_path: Path,
    *,
    cleaner: OwnedGroupCleaner | None = None,
) -> CleanupReport | None:
    """Clean a previous interrupted run without trusting process names."""
    registry = ProcessRegistry(state_path)
    run_token = registry.read_token()
    if run_token is None:
        return None
    active_cleaner = cleaner or OwnedGroupCleaner()
    report = active_cleaner.cleanup(run_token)
    if report.clean:
        registry.clear_if_token(run_token)
    return report
