"""
Gazebo world metadata helpers.

Responsibility:
- Read Gazebo world metadata needed by launch-time bridge configuration.
- Derive the exact world-scoped clock topic from an SDF world file.

Does not own:
- World model editing, bridge processes, or Gazebo process actions.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


def gazebo_clock_topic(world_path: Path) -> str:
    """Return the Gazebo clock topic derived from an SDF world name."""
    try:
        root = ET.parse(world_path).getroot()
    except (ET.ParseError, OSError) as error:
        raise ValueError(f"Cannot read Gazebo world SDF: {world_path}") from error

    world = root if root.tag == "world" else root.find("world")
    if world is None:
        raise ValueError(f"Gazebo SDF has no <world> element: {world_path}")
    world_name = world.attrib.get("name", "").strip()
    if not world_name:
        raise ValueError(f"Gazebo <world> has no non-empty name: {world_path}")
    return f"/world/{world_name}/clock"
