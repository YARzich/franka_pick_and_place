#!/usr/bin/env python3


"""
Persistent JSON-lines worker entrypoint for MegaPose inference.

Responsibility:
- Read JSON requests from stdin and write JSON responses to stdout.
- Handle bad JSON and shutdown commands.
- Delegate MegaPose runtime execution to `pose_megapose_worker_runtime`.

Does not own:
- ROS subscriptions, publishers or parameters.
- Runtime example file creation.
- MegaPose model cache, coarse tensor conversion or output serialization.
"""

from __future__ import annotations

from pathlib import Path
import sys


def _add_source_package_root_to_path() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "manipulation_perception").is_dir():
            parent_text = str(parent)
            if parent_text not in sys.path:
                sys.path.insert(0, parent_text)
            common_root = parent.parent / "manipulation_perception"
            if common_root.is_dir() and str(common_root) not in sys.path:
                sys.path.insert(0, str(common_root))
            return


_add_source_package_root_to_path()

from manipulation_perception.pose.pose_megapose_worker_runtime import (  # noqa: E402
    configure_megapose_worker_logging,
    MegaPoseRuntimeBackend,
)
from manipulation_perception.pose.worker_protocol import (  # noqa: E402,I100,I101,I201
    run_jsonl_worker_loop,
)


def run_worker_loop(worker: MegaPoseRuntimeBackend) -> int:
    """Serve serialized MegaPose requests until EOF or shutdown."""
    return run_jsonl_worker_loop(worker)


def main() -> int:
    """Configure worker-safe logging, then own and run the MegaPose backend."""
    configure_megapose_worker_logging()
    return run_worker_loop(MegaPoseRuntimeBackend())


if __name__ == "__main__":
    raise SystemExit(main())
