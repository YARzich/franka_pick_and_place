#!/usr/bin/env python3


"""
Persistent JSON-lines worker entrypoint for GigaPose coarse retrieval.

Responsibility:
- Read JSON requests from stdin and write JSON responses to stdout.
- Handle bad JSON and shutdown commands.
- Delegate GigaPose retrieval runtime to `pose_gigapose_worker_runtime`.

Does not own:
- ROS subscriptions, publishers or parameters.
- Runtime dataset/input preparation.
- GigaPose model loading, retrieval batch construction or prediction parsing.
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

from manipulation_perception.pose.pose_gigapose_worker_runtime import (  # noqa: E402
    GigaPoseWorker,
)
from manipulation_perception.pose.worker_protocol import (  # noqa: E402,I100,I101,I201
    run_jsonl_worker_loop,
)


def run_worker_loop(worker: GigaPoseWorker) -> int:
    """Serve serialized coarse-retrieval requests until EOF or shutdown."""
    return run_jsonl_worker_loop(worker)


def main() -> int:
    """Create the process-owned GigaPose worker and serve stdin requests."""
    return run_worker_loop(GigaPoseWorker())


if __name__ == "__main__":
    raise SystemExit(main())
