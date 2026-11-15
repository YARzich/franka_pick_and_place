"""
Compose the Franka active-cancel runtime validation client process.

Responsibility:
- Select the active-cancel probe runner and stable ROS node name.

Does not own:
- Scenario goals, action transport, cancellation policy, or evidence checks.
"""

from __future__ import annotations

from franka_motion_runtime.tasks.active_cancel_probe import (
    run_active_cancel_probe,
)
from franka_motion_runtime.tasks.sequence_client_process import run_client_process


def main(args=None) -> int:
    """Run the active-cancel probe through the shared contained process shell."""
    return run_client_process(
        "franka_transfer_sequence_active_cancel_probe",
        run_active_cancel_probe,
        args=args,
    )
