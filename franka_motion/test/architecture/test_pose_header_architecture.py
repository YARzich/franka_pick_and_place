"""
Protect the public boundary between pure pose geometry and ROS/TF adapters.

Responsibility:
- Keep pure pose consumers independent from ROS logging and TF buffer headers.
- Preserve the historical aggregate include while exposing focused public
  headers.
"""

from pathlib import Path


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_INCLUDE_ROOT = _PACKAGE_ROOT / 'include' / 'franka_motion'
_PURE_HEADER = _INCLUDE_ROOT / 'pose_geometry.hpp'


def _source(path: Path) -> str:
    """Read one source-controlled boundary file."""
    return path.read_text(encoding='utf-8')


def test_pure_pose_header_avoids_ros_runtime_and_tf_buffer() -> None:
    """Keep geometry independent from ROS nodes, logging, and TF buffers."""
    source = _source(_PURE_HEADER)

    for forbidden_include in (
        '"rclcpp/',
        '"tf2_ros/',
        '"tf2_geometry_msgs/',
    ):
        assert forbidden_include not in source


def test_compatibility_header_aggregates_both_focused_public_headers() -> None:
    """Existing pose_utils users must retain every historical helper name."""
    source = _source(_INCLUDE_ROOT / 'pose_utils.hpp')

    assert '#include "franka_motion/pose_geometry.hpp"' in source
    assert '#include "franka_motion/pose_ros_utils.hpp"' in source


def test_v2_detection_buffer_stays_private() -> None:
    """Do not reinstall the scenario-specific observation cache as public API."""
    private_header = (
        _PACKAGE_ROOT / 'src' / 'workflow_v2' / 'detected_pose_buffer.hpp'
    )

    assert private_header.is_file()
    assert not (_INCLUDE_ROOT / 'detected_pose_buffer.hpp').exists()
