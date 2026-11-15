#!/usr/bin/env python3
"""
Managed wrapper around ros_gz_bridge parameter_bridge.

Responsibility:
- Build and start the parameter_bridge command from wrapper CLI args.
- Stop the bridge with SIGTERM and optional SIGKILL fallback on launch shutdown.
- Validate bridge executable and config paths before spawning.

Does not own:
- Bridge YAML contents.
- Gazebo or ROS topic semantics.
- Other launched process cleanup.
"""

from __future__ import annotations

import argparse
from pathlib import Path
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
    # Keep direct execution valid when a launch environment replaces PYTHONPATH.
    import process_group_runner as runner_module

ProcessGroupRunner = runner_module.ProcessGroupRunner
ProcessGroupRunnerConfig = runner_module.ProcessGroupRunnerConfig


class BridgeWrapper:
    """Adapt bridge-specific CLI values to the generic process runner."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Store validated bridge arguments for command construction."""
        self._args = args

    def _build_command(self) -> list[str]:
        return [
            self._args.bridge_executable,
            "--ros-args",
            "-r",
            f"__node:={self._args.bridge_name}",
            "--log-level",
            self._args.log_level,
            "-p",
            f"config_file:={self._args.config_file}",
        ]

    def _build_runner_config(self) -> ProcessGroupRunnerConfig:
        return ProcessGroupRunnerConfig(
            command=self._build_command(),
            child_name="ros_gz_bridge_wrapper",
            shutdown_signal=signal.SIGTERM,
            shutdown_timeout_sec=self._args.term_timeout_sec,
            kill_timeout_sec=self._args.kill_timeout_sec,
            cleanup_on_child_exit=False,
        )

    def run(self) -> int:
        """Run the parameter bridge under bounded process-group ownership."""
        return ProcessGroupRunner(self._build_runner_config()).run()


def parse_args() -> argparse.Namespace:
    """Parse and validate the stable bridge-wrapper command line."""
    parser = argparse.ArgumentParser(
        description="Wrapper around ros_gz_bridge parameter_bridge with controlled shutdown.",
    )
    parser.add_argument(
        "--bridge-executable",
        default="/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge",
        help="Absolute path to ros_gz_bridge parameter_bridge executable.",
    )
    parser.add_argument(
        "--bridge-name",
        required=True,
        help="ROS node name for the spawned bridge.",
    )
    parser.add_argument(
        "--config-file",
        required=True,
        help="Path to ros_gz_bridge YAML config file.",
    )
    parser.add_argument(
        "--log-level",
        default="info",
        help="ROS log level passed to parameter_bridge.",
    )
    parser.add_argument(
        "--term-timeout-sec",
        type=float,
        default=3.0,
        help="How long to wait after SIGTERM before forcing SIGKILL.",
    )
    parser.add_argument(
        "--kill-timeout-sec",
        type=float,
        default=2.0,
        help="How long to wait after SIGKILL before letting the wrapper exit.",
    )
    args, _unknown = parser.parse_known_args()

    if not Path(args.bridge_executable).exists():
        raise FileNotFoundError(f"Bridge executable not found: {args.bridge_executable}")
    if not Path(args.config_file).exists():
        raise FileNotFoundError(f"Bridge config file not found: {args.config_file}")
    if args.term_timeout_sec < 0.0:
        parser.error("--term-timeout-sec must be >= 0")
    if args.kill_timeout_sec < 0.0:
        parser.error("--kill-timeout-sec must be >= 0")
    return args


def main() -> int:
    """Run the validated bridge wrapper and map failures to an exit code."""
    try:
        wrapper = BridgeWrapper(parse_args())
        return wrapper.run()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"ros_gz_bridge_wrapper error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
