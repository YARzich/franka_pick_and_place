"""
Test canonical Franka xacro resources without starting a ROS graph.

Responsibility: Verify installed resources, TF geometry, joint contracts, and backend support.
"""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest


PACKAGE_SHARE = Path(get_package_share_directory('franka_motion'))
DESCRIPTION_ROOT = PACKAGE_SHARE / 'description'
TEST_ROOT = Path(__file__).resolve().parent

EXPECTED_LINKS = [
    'panda_grasp_frame',
    'panda_hand',
    'panda_leftfinger',
    'panda_link0',
    'panda_link1',
    'panda_link2',
    'panda_link3',
    'panda_link4',
    'panda_link5',
    'panda_link6',
    'panda_link7',
    'panda_link8',
    'panda_rightfinger',
]
EXPECTED_JOINTS = [
    'panda_finger_joint1',
    'panda_finger_joint2',
    'panda_grasp_fixed',
    'panda_hand_joint',
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
    'panda_joint8',
]


def test_canonical_xacro_resources_are_installed() -> None:
    """Keep package URIs stable for downstream robot-description consumers."""
    expected_paths = [
        'urdf/panda_gazebo.urdf.xacro',
        'urdf/panda_isaac.urdf.xacro',
        'urdf/panda_nomimic.urdf.xacro',
        'ros2_control/panda.ros2_control.xacro',
        'ros2_control/panda_hand.ros2_control.xacro',
    ]

    for relative_path in expected_paths:
        assert (DESCRIPTION_ROOT / relative_path).is_file()


def test_nomimic_geometry_preserves_tf_contract() -> None:
    """Validate the simulator-independent tree and explicit finger joints."""
    robot = _expand(DESCRIPTION_ROOT / 'urdf' / 'panda_nomimic.urdf.xacro')

    assert robot.attrib['name'] == 'panda'
    assert sorted(link.attrib['name'] for link in robot.findall('link')) == EXPECTED_LINKS
    assert sorted(joint.attrib['name'] for joint in robot.findall('joint')) == EXPECTED_JOINTS
    assert _root_links(robot) == ['panda_link0']
    assert robot.find("./link[@name='world']") is None
    assert not robot.findall('.//mimic')

    grasp_joint = _required(robot, "./joint[@name='panda_grasp_fixed']")
    assert _required(grasp_joint, 'parent').attrib == {'link': 'panda_hand'}
    assert _required(grasp_joint, 'child').attrib == {'link': 'panda_grasp_frame'}
    assert _required(grasp_joint, 'origin').attrib == {
        'rpy': '0 0 0',
        'xyz': '0 0 0.07',
    }


def test_gazebo_assembly_preserves_fixed_base_and_split_control() -> None:
    """Validate Gazebo world anchoring and independent arm/hand ownership."""
    robot = _expand(DESCRIPTION_ROOT / 'urdf' / 'panda_gazebo.urdf.xacro')

    assert _root_links(robot) == ['world']
    world_joint = _required(robot, "./joint[@name='panda_world_joint']")
    assert _required(world_joint, 'parent').attrib == {'link': 'world'}
    assert _required(world_joint, 'child').attrib == {'link': 'panda_link0'}
    assert _control_plugins(robot) == {
        'PandaGazeboSystem': 'gz_ros2_control/GazeboSimSystem',
        'PandaHandGazeboSystem': 'gz_ros2_control/GazeboSimSystem',
    }


def test_isaac_assembly_preserves_fixed_base_and_unified_control() -> None:
    """Validate Isaac world anchoring and ownership of its unified joint stream."""
    robot = _expand(DESCRIPTION_ROOT / 'urdf' / 'panda_isaac.urdf.xacro')

    assert _root_links(robot) == ['world']
    world_joint = _required(robot, "./joint[@name='panda_world_joint']")
    assert _required(world_joint, 'parent').attrib == {'link': 'world'}
    assert _required(world_joint, 'child').attrib == {'link': 'panda_link0'}
    assert _control_plugins(robot) == {
        'PandaIsaacSystem': 'topic_based_ros2_control/TopicBasedSystem',
    }


def test_arm_control_macro_preserves_joint_and_topic_contract(tmp_path: Path) -> None:
    """Expand the public arm macro with the topic-based backend."""
    invocation = f"""
  <xacro:panda_ros2_control
    name="TestArmSystem"
    hardware_plugin="topic_based_ros2_control/TopicBasedSystem"
    initial_positions_file="{TEST_ROOT / 'initial_positions.yaml'}"
    command_topic="/test/commands"
    joint_state_topic="/test/states"
    include_hand="true" />
"""
    robot = _expand_macro(
        tmp_path,
        DESCRIPTION_ROOT / 'ros2_control' / 'panda.ros2_control.xacro',
        invocation,
    )
    control = _required(robot, "./ros2_control[@name='TestArmSystem']")

    assert _required(control, './hardware/plugin').text == (
        'topic_based_ros2_control/TopicBasedSystem'
    )
    assert _hardware_params(control) == {
        'joint_commands_topic': '/test/commands',
        'joint_states_topic': '/test/states',
    }
    assert [joint.attrib['name'] for joint in control.findall('joint')] == [
        *(f'panda_joint{index}' for index in range(1, 8)),
        'panda_finger_joint1',
        'panda_finger_joint2',
    ]
    assert _initial_position(control, 'panda_joint1') == '0.1'
    assert _initial_position(control, 'panda_joint7') == '-0.7'


@pytest.mark.parametrize(
    ('hardware_type', 'plugin', 'expected_params'),
    [
        ('mock_components', 'mock_components/GenericSystem', {}),
        (
            'isaac',
            'topic_based_ros2_control/TopicBasedSystem',
            {
                'joint_commands_topic': '/test/hand_commands',
                'joint_states_topic': '/test/hand_states',
            },
        ),
        ('gz', 'gz_ros2_control/GazeboSimSystem', {}),
    ],
)
def test_hand_control_macro_supports_declared_backends(
    tmp_path: Path,
    hardware_type: str,
    plugin: str,
    expected_params: dict[str, str],
) -> None:
    """Keep backend selection and finger initialization deterministic."""
    invocation = f"""
  <xacro:panda_hand_ros2_control
    name="TestHandSystem"
    ros2_control_hardware_type="{hardware_type}"
    command_topic="/test/hand_commands"
    joint_state_topic="/test/hand_states"
    initial_position="0.012" />
"""
    robot = _expand_macro(
        tmp_path,
        DESCRIPTION_ROOT / 'ros2_control' / 'panda_hand.ros2_control.xacro',
        invocation,
    )
    control = _required(robot, "./ros2_control[@name='TestHandSystem']")

    assert _required(control, './hardware/plugin').text == plugin
    assert _hardware_params(control) == expected_params
    assert [joint.attrib['name'] for joint in control.findall('joint')] == [
        'panda_finger_joint1',
        'panda_finger_joint2',
    ]
    assert _initial_position(control, 'panda_finger_joint1') == '0.012'
    assert _initial_position(control, 'panda_finger_joint2') == '0.012'


def _expand(path: Path) -> ET.Element:
    """Expand a xacro file and return its XML root."""
    result = subprocess.run(
        ['xacro', str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _expand_macro(tmp_path: Path, include_path: Path, invocation: str) -> ET.Element:
    """Create a minimal robot which invokes one macro from an installed file."""
    wrapper = tmp_path / f'{include_path.stem}.test.urdf.xacro'
    wrapper.write_text(
        '<?xml version="1.0"?>\n'
        '<robot name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        f'  <xacro:include filename="{include_path}" />\n'
        f'{invocation}'
        '</robot>\n',
        encoding='utf-8',
    )
    return _expand(wrapper)


def _required(root: ET.Element, path: str) -> ET.Element:
    """Return a required XML element with a useful assertion on failure."""
    element = root.find(path)
    assert element is not None, f'missing XML element: {path}'
    return element


def _root_links(robot: ET.Element) -> list[str]:
    """Return links which are not children of any joint."""
    links = {link.attrib['name'] for link in robot.findall('link')}
    children = {
        child.attrib['link']
        for joint in robot.findall('joint')
        for child in joint.findall('child')
    }
    return sorted(links - children)


def _hardware_params(control: ET.Element) -> dict[str, str]:
    """Return hardware parameters from one ros2_control block."""
    return {
        parameter.attrib['name']: parameter.text or ''
        for parameter in control.findall('./hardware/param')
    }


def _control_plugins(robot: ET.Element) -> dict[str, str]:
    """Return the hardware plugin selected for each ros2_control system."""
    return {
        control.attrib['name']: _required(control, './hardware/plugin').text or ''
        for control in robot.findall('ros2_control')
    }


def _initial_position(control: ET.Element, joint_name: str) -> str:
    """Return the initial position declared for one controlled joint."""
    value = _required(
        control,
        f"./joint[@name='{joint_name}']/state_interface[@name='position']"
        "/param[@name='initial_value']",
    )
    assert value.text is not None
    return value.text
