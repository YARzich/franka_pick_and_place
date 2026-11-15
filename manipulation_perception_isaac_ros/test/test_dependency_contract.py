"""Responsibility: Verify the generic Isaac ROS dependency-only package contract."""

from pathlib import Path
import xml.etree.ElementTree as ET


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_ISAAC_ROS_PACKAGES = {
    'isaac_ros_ess',
    'isaac_ros_foundationpose',
    'isaac_ros_image_proc',
    'isaac_ros_stereo_image_proc',
}


def _manifest_root():
    """Return the parsed package manifest root."""
    return ET.parse(_PACKAGE_ROOT / 'package.xml').getroot()


def _exec_dependencies():
    """Return executable dependencies indexed by package name."""
    return {
        element.text.strip(): element
        for element in _manifest_root().findall('exec_depend')
        if element.text
    }


def test_package_identity_is_robot_independent() -> None:
    """The package name and dependency graph must not encode one robot."""
    root = _manifest_root()
    dependencies = _exec_dependencies()

    assert root.findtext('name') == 'manipulation_perception_isaac_ros'
    assert 'manipulation_perception' in dependencies
    assert not any(name.startswith('franka_motion') for name in dependencies)


def test_manifest_pins_isaac_ros_components_to_compatible_minor() -> None:
    """All constrained NVIDIA components must share one tested minor line."""
    dependencies = _exec_dependencies()

    assert _ISAAC_ROS_PACKAGES <= dependencies.keys()
    for package_name in _ISAAC_ROS_PACKAGES:
        dependency = dependencies[package_name]
        assert dependency.attrib == {
            'version_gte': '4.1.0',
            'version_lt': '4.2.0',
        }


def test_bundle_contains_no_runtime_implementation() -> None:
    """Scenario orchestration and runtime code must stay in their owning packages."""
    for directory in ('config', 'include', 'launch', 'scripts', 'src'):
        assert not (_PACKAGE_ROOT / directory).exists()
