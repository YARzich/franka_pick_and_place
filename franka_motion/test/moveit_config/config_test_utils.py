"""
Provide shared parsers and stable Panda identifiers for configuration tests.

Responsibility: Load canonical URDF, SRDF, and YAML resources for cross-file assertions.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml


PACKAGE_ROOT = Path(__file__).parents[2]
CONFIG_DIR = PACKAGE_ROOT / 'config' / 'moveit'
ARM_JOINTS = tuple(f'panda_joint{index}' for index in range(1, 8))
FINGER_JOINTS = ('panda_finger_joint1', 'panda_finger_joint2')


def load_yaml(name: str):
    """Load one canonical YAML resource without applying ROS parameter scope."""
    return yaml.safe_load((CONFIG_DIR / name).read_text(encoding='utf-8'))


def load_package_yaml(relative_path: str):
    """Load package metadata outside the nested MoveIt configuration directory."""
    return yaml.safe_load(
        (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8')
    )


def load_urdf() -> ET.Element:
    """Expand the canonical xacro wrapper and return its URDF XML root."""
    document = xacro.process_file(str(CONFIG_DIR / 'panda.urdf.xacro'))
    return ET.fromstring(document.toxml())


def load_srdf() -> ET.Element:
    """Return the canonical semantic model root."""
    return ET.parse(CONFIG_DIR / 'panda.srdf').getroot()
