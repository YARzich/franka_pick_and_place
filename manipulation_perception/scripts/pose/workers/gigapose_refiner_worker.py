#!/usr/bin/env python3


"""
Persistent JSON-lines worker entrypoint for GigaPose/MegaPose refinement.

Responsibility:
- Read JSON requests from stdin and write JSON responses to stdout.
- Handle bad JSON, shutdown commands and backend close on exit.
- Delegate refiner/scoring runtime to `pose_gigapose_refiner_worker_runtime`.

Does not own:
- Coarse retrieval.
- ROS subscriptions, publishers or parameters.
- Runtime model preparation, refiner loading or object_data serialization.
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

from manipulation_perception.pose.pose_gigapose_refiner_worker_runtime import (  # noqa: E402
    GigaPoseRefinerWorker,
)
from manipulation_perception.pose.worker_protocol import (  # noqa: E402,I100,I101,I201
    run_jsonl_worker_loop,
)


def run_worker_loop(worker: GigaPoseRefinerWorker) -> int:
    """Serve serialized refiner requests and close cached GPU backends on exit."""
    return run_jsonl_worker_loop(worker, close_worker=True)


def main() -> int:
    """Create the process-owned refiner worker and serve stdin until shutdown."""
    return run_worker_loop(GigaPoseRefinerWorker())


if __name__ == "__main__":
    raise SystemExit(main())
