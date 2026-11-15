"""
Test Franka Gazebo resources and bridge contracts without simulation.

Responsibility:
- Verify world parsing, local assets, simulated-time QoS, camera topics, frames,
  and table geometry owned by ``franka_motion``.
"""

from pathlib import Path
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

import pytest
import yaml

from franka_motion_runtime.simulation.gazebo.bridge import write_clock_bridge_config
from franka_motion_runtime.simulation.gazebo.table_camera import (
    write_table_camera_bridge_config,
)
from franka_motion_runtime.simulation.gazebo.world import gazebo_clock_topic

_PACKAGE_ROOT = Path(__file__).resolve().parents[3]
_DESCRIPTION_ROOT = _PACKAGE_ROOT / "description" / "gazebo"
_WORLD_ROOT = _PACKAGE_ROOT / "worlds" / "gazebo"
_WORLD_FILES = (
    "panda_empty.world.sdf",
    "table_world.sdf",
    "table_world_camera.sdf",
)
_LOCAL_ASSET_TAGS = {
    "albedo_map",
    "emissive_map",
    "environment_map",
    "light_map",
    "metalness_map",
    "normal_map",
    "roughness_map",
    "uri",
}


def _local_asset_path(sdf_path: Path, uri: str) -> Path | None:
    """Resolve only filesystem and ``franka_motion`` package resource references."""
    parsed = urlparse(uri)
    if parsed.scheme == "file":
        return Path(parsed.path)
    if parsed.scheme == "package" and parsed.netloc == "franka_motion":
        return _PACKAGE_ROOT / parsed.path.lstrip("/")
    if parsed.scheme:
        return None

    path = Path(uri)
    return path if path.is_absolute() else sdf_path.parent / path


def test_worlds_are_parseable_and_expose_world_scoped_clock_topics() -> None:
    """Every canonical world must be valid XML with a deterministic clock topic."""
    for world_name in _WORLD_FILES:
        world_path = _WORLD_ROOT / world_name
        world = ET.parse(world_path).getroot().find("world")

        assert world is not None
        assert gazebo_clock_topic(world_path) == f"/world/{world.attrib['name']}/clock"


def test_sdf_local_asset_references_exist() -> None:
    """Canonical SDF must not contain dangling package-local texture or mesh URIs."""
    sdf_paths = [_WORLD_ROOT / world_name for world_name in _WORLD_FILES]
    sdf_paths.append(_DESCRIPTION_ROOT / "table_camera.sdf")

    for sdf_path in sdf_paths:
        root = ET.parse(sdf_path).getroot()
        for element in root.iter():
            tag = element.tag.rsplit("}", maxsplit=1)[-1]
            if tag not in _LOCAL_ASSET_TAGS or not element.text:
                continue
            uri = element.text.strip()
            local_path = _local_asset_path(sdf_path, uri)
            if local_path is not None:
                assert local_path.is_file(), f"{sdf_path}: missing local asset {uri}"


def test_clock_bridge_keeps_ros_clock_qos(tmp_path: Path) -> None:
    """The bridge must retain ROS simulated-time semantics."""
    config_path = tmp_path / "clock.yaml"
    write_clock_bridge_config(config_path, "/world/test/clock")

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))

    assert config == [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": "/world/test/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS",
            "lazy": False,
            "qos_profile": "CLOCK",
        }
    ]


def test_table_camera_bridge_adds_both_camera_info_topics(tmp_path: Path) -> None:
    """Stereo calibration topics must be bridged without changing their names."""
    config_path = tmp_path / "camera.yaml"
    write_table_camera_bridge_config(config_path, "/world/table/clock")

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    topics = [entry["ros_topic_name"] for entry in config]

    assert topics == [
        "/clock",
        "/table_stereo/left/camera_info",
        "/table_stereo/right/camera_info",
    ]


def test_table_camera_sdf_uses_rep103_optical_frames() -> None:
    """Sensor messages must identify optical frames, not physical links."""
    root = ET.parse(_DESCRIPTION_ROOT / "table_camera.sdf").getroot()
    frames = {
        sensor.attrib["name"]: sensor.findtext("gz_frame_id")
        for sensor in root.findall(".//sensor")
    }

    assert frames == {
        "table_stereo_left_camera": "table_stereo_left_optical_frame",
        "table_stereo_right_camera": "table_stereo_right_optical_frame",
    }


def test_table_camera_world_keeps_textured_box_table() -> None:
    """Stereo needs visual texture while collision stays deterministic."""
    world_path = _WORLD_ROOT / "table_world_camera.sdf"
    world = ET.parse(world_path).getroot().find("world")
    assert world is not None
    table = next(
        model
        for model in world.findall("model")
        if model.attrib.get("name") == "Reflective table"
    )

    collision_size = table.findtext("./link/collision/geometry/box/size")
    visual_size = table.findtext("./link/visual/geometry/box/size")
    albedo_map = table.findtext("./link/visual/material/pbr/metal/albedo_map")

    assert collision_size == visual_size == "1.5532 2.3937 0.05"
    assert albedo_map == "materials/textures/table_stereo_albedo.png"
    assert (world_path.parent / albedo_map).is_file()


@pytest.mark.parametrize(
    "sdf",
    [
        "<sdf>",
        "<sdf version='1.9'><model name='not_a_world'/></sdf>",
        "<sdf version='1.9'><world name='  '/></sdf>",
    ],
)
def test_clock_topic_rejects_malformed_or_unnamed_worlds(
    tmp_path: Path,
    sdf: str,
) -> None:
    """Invalid worlds must fail explicitly instead of raising AttributeError."""
    world_path = tmp_path / "invalid.sdf"
    world_path.write_text(sdf, encoding="utf-8")

    with pytest.raises(ValueError, match="Gazebo"):
        gazebo_clock_topic(world_path)
