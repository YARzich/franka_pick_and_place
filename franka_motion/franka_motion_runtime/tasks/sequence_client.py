"""
Compose the canonical indexed-parameter transfer-sequence client.

Responsibility:
- Select the indexed request loader and stable client node name.

Does not own:
- Process lifecycle, action transport, rigid resolution, or motion execution.
"""

from __future__ import annotations

from franka_motion_runtime.tasks.sequence_client_process import run_client_process
from franka_motion_runtime.tasks.sequence_client_runtime import run_sequence_client


def main(args=None) -> int:
    """Run the canonical client through the shared contained process shell."""
    return run_client_process(
        "franka_transfer_sequence_client",
        run_sequence_client,
        args=args,
    )
