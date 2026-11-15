"""
Command construction for the installed run_franka_isaac_sim.py tool.

Responsibility:
- Translate Isaac scenario settings into CLI arguments for the simulator script.
- Keep topic, robot asset and scene flags consistent across Isaac launch entrypoints.

Does not own:
- Starting the process or setting environment variables.
- Simulator runtime behavior inside run_franka_isaac_sim.py.
"""

from __future__ import annotations

from pathlib import Path

from .contracts import (
    ISAAC_JOINT_COMMANDS_TOPIC,
    ISAAC_JOINT_STATES_TOPIC,
    IsaacLaunchPathsLike,
)
from .scenarios import IsaacMoveItScenario


def create_isaac_command(
    *,
    paths: IsaacLaunchPathsLike,
    scenario: IsaacMoveItScenario,
    isaac_python: str,
    isaac_prim: str,
    headless: bool,
    fix_base: bool,
    time_steps: str,
    isaac_experience: str,
) -> list[str]:
    """Build the canonical Isaac runner command without starting a process."""
    isaac_cmd = [
        isaac_python,
        str(paths.isaac_script),
        "--robot-usd",
        str(paths.panda_usd),
        "--scene-prim",
        isaac_prim,
        "--command-topic",
        ISAAC_JOINT_COMMANDS_TOPIC,
        "--state-topic",
        ISAAC_JOINT_STATES_TOPIC,
    ]
    if scenario.scene_usd_file is not None:
        isaac_cmd.extend(
            [
                "--scene-usd",
                str(_isaac_worlds_path(paths) / scenario.scene_usd_file),
            ]
        )
    if scenario.no_default_ground:
        isaac_cmd.append("--no-default-ground")
    if scenario.add_cube:
        isaac_cmd.extend(
            [
                "--add-cube",
                "--cube-size",
                str(scenario.cube_size),
                "--cube-position",
                *map(str, scenario.cube_position),
            ]
        )
    if headless:
        isaac_cmd.append("--headless")
    if scenario.include_isaac_experience_arg and isaac_experience:
        isaac_cmd.extend(["--experience", isaac_experience])
    if fix_base:
        isaac_cmd.append("--fix-base")
    if time_steps not in {"0", ""}:
        isaac_cmd.extend(["--time-steps", time_steps])
    return isaac_cmd


def _isaac_worlds_path(paths: IsaacLaunchPathsLike) -> Path:
    """Return the scenario package's canonical Isaac-world directory."""
    return Path(paths.isaac_worlds)
