"""
Test public robot-description aliases and fixed-base frame semantics.

Responsibility: Verify local xacro aliases, REP-103 camera frames, SRDF, and joint limits.
"""

from collections import Counter
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_PACKAGE_SHARE = Path(get_package_share_directory('franka_motion'))
_DESCRIPTION_ROOT = _PACKAGE_SHARE / 'description'


def test_table_camera_sensor_messages_use_rep103_optical_frames():
    model = ET.parse(
        _PACKAGE_ROOT / 'description' / 'gazebo' / 'table_camera.sdf'
    ).getroot()
    sensor_frames = {
        sensor.attrib['name']: sensor.findtext('gz_frame_id')
        for sensor in model.findall('.//sensor')
    }

    assert sensor_frames == {
        'table_stereo_left_camera': 'table_stereo_left_optical_frame',
        'table_stereo_right_camera': 'table_stereo_right_optical_frame',
    }


def test_public_geometry_path_matches_canonical_description():
    """Keep the historical package URI as a semantics-preserving local alias."""
    public = _generate_xacro(
        _DESCRIPTION_ROOT / 'panda_nomimic.urdf.xacro'
    )
    canonical = _generate_xacro(
        _DESCRIPTION_ROOT / 'urdf' / 'panda_nomimic.urdf.xacro'
    )

    assert _xml_signature(public) == _xml_signature(canonical)


def test_public_arm_control_macro_matches_canonical_description(tmp_path: Path):
    """Prove that the public alias exposes the canonical arm macro output."""
    invocation = f"""
  <xacro:panda_ros2_control
    name="CompatibilityArmSystem"
    hardware_plugin="topic_based_ros2_control/TopicBasedSystem"
    initial_positions_file="{_PACKAGE_ROOT / 'config' / 'control' / 'initial_positions.yaml'}"
    command_topic="/compatibility/commands"
    joint_state_topic="/compatibility/states"
    include_hand="true" />
"""
    public = _expand_macro(
        tmp_path,
        _DESCRIPTION_ROOT / 'panda.ros2_control.xacro',
        invocation,
    )
    canonical = _expand_macro(
        tmp_path,
        _DESCRIPTION_ROOT / 'ros2_control' / 'panda.ros2_control.xacro',
        invocation,
    )

    assert _xml_signature(public) == _xml_signature(canonical)


def test_public_hand_control_macro_matches_canonical_description(tmp_path: Path):
    """Prove that the public alias exposes the canonical hand macro output."""
    invocation = """
  <xacro:panda_hand_ros2_control
    name="CompatibilityHandSystem"
    ros2_control_hardware_type="isaac"
    command_topic="/compatibility/hand_commands"
    joint_state_topic="/compatibility/hand_states"
    initial_position="0.02" />
"""
    public = _expand_macro(
        tmp_path,
        _DESCRIPTION_ROOT / 'panda_hand.ros2_control.xacro',
        invocation,
    )
    canonical = _expand_macro(
        tmp_path,
        _DESCRIPTION_ROOT / 'ros2_control' / 'panda_hand.ros2_control.xacro',
        invocation,
    )

    assert _xml_signature(public) == _xml_signature(canonical)


def test_gazebo_description_keeps_fixed_world_base():
    robot = _generate_robot_description('description/panda_gazebo.urdf.xacro')

    assert _root_links(robot) == ['world']
    assert _world_joint(robot).attrib == {
        'name': 'panda_world_joint',
        'type': 'fixed',
    }
    origin = _world_joint(robot).find('origin')
    assert origin is not None
    assert origin.attrib == {
        'rpy': '0.0 0.0 0.0',
        'xyz': '0.0 0.0 1.0',
    }
    assert _ros2_control_plugins(robot) == {
        'PandaGazeboSystem': 'gz_ros2_control/GazeboSimSystem',
        'PandaHandGazeboSystem': 'gz_ros2_control/GazeboSimSystem',
    }
    finger1 = robot.find("./joint[@name='panda_finger_joint1']")
    finger2 = robot.find("./joint[@name='panda_finger_joint2']")
    assert finger1 is not None
    assert finger2 is not None
    assert finger1.find('mimic') is None
    assert finger2.find('mimic') is None
    hand_control = robot.find("./ros2_control[@name='PandaHandGazeboSystem']")
    assert hand_control is not None
    assert hand_control.find("./joint[@name='panda_finger_joint1']/command_interface") is not None
    assert hand_control.find("./joint[@name='panda_finger_joint2']/command_interface") is not None
    assert _joint_initial_position(hand_control, 'panda_finger_joint1') == '0.035'
    assert _joint_initial_position(hand_control, 'panda_finger_joint2') == '0.035'


def test_isaac_description_keeps_fixed_world_base_and_topic_control():
    robot = _generate_robot_description('description/panda_isaac.urdf.xacro')

    assert _root_links(robot) == ['world']
    assert _world_joint(robot).attrib == {
        'name': 'panda_world_joint',
        'type': 'fixed',
    }
    assert _world_joint(robot).find('origin') is None
    assert _ros2_control_plugins(robot) == {
        'PandaIsaacSystem': 'topic_based_ros2_control/TopicBasedSystem',
    }
    finger2 = robot.find("./joint[@name='panda_finger_joint2']")
    assert finger2 is not None
    assert finger2.find('mimic') is None


def test_moveit_semantic_description_matches_fixed_base_urdf():
    srdf = ET.parse(_PACKAGE_ROOT / 'config' / 'panda_no_virtual_joint.srdf').getroot()

    assert srdf.attrib['name'] == 'panda'
    assert srdf.find('virtual_joint') is None
    arm_chain = srdf.find("./group[@name='panda_arm']/chain")
    assert arm_chain is not None
    assert arm_chain.attrib == {
        'base_link': 'panda_link0',
        'tip_link': 'panda_link8',
    }
    end_effector = srdf.find("./end_effector[@name='hand']")
    assert end_effector is not None
    assert end_effector.attrib == {
        'group': 'hand',
        'name': 'hand',
        'parent_group': 'panda_arm',
        'parent_link': 'panda_link8',
    }


def test_local_semantic_description_matches_upstream_except_base():
    upstream_srdf = ET.parse(
        Path(get_package_share_directory('moveit_resources_panda_moveit_config'))
        / 'config'
        / 'panda.srdf'
    ).getroot()
    local_srdf = ET.parse(_PACKAGE_ROOT / 'config' / 'panda_no_virtual_joint.srdf').getroot()

    assert len(upstream_srdf.findall('virtual_joint')) == 1
    assert len(local_srdf.findall('virtual_joint')) == 0
    assert _semantic_signature(local_srdf) == _semantic_signature(upstream_srdf)


def _joint_initial_position(control: ET.Element, joint_name: str) -> str:
    initial_value = control.find(
        f"./joint[@name='{joint_name}']/state_interface[@name='position']/param[@name='initial_value']"
    )
    assert initial_value is not None
    assert initial_value.text is not None
    return initial_value.text


def _generate_robot_description(relative_path: str) -> ET.Element:
    canonical_path = _DESCRIPTION_ROOT / 'urdf' / Path(relative_path).name
    return _generate_xacro(canonical_path)


def _generate_xacro(path: Path) -> ET.Element:
    command = ['xacro', str(path)]
    xml = subprocess.check_output(
        command,
        text=True,
    )
    return ET.fromstring(xml)


def _expand_macro(tmp_path: Path, include_path: Path, invocation: str) -> ET.Element:
    """Invoke one macro from either the canonical file or public alias."""
    parent_label = include_path.parent.name.replace('.', '_')
    wrapper = tmp_path / f'{parent_label}_{include_path.name}.urdf.xacro'
    wrapper.write_text(
        '<?xml version="1.0"?>\n'
        '<robot name="compatibility" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        f'  <xacro:include filename="{include_path}" />\n'
        f'{invocation}'
        '</robot>\n',
        encoding='utf-8',
    )
    return _generate_xacro(wrapper)


def _root_links(robot: ET.Element) -> list[str]:
    links = {link.attrib['name'] for link in robot.findall('link')}
    child_links = {
        child.attrib['link']
        for joint in robot.findall('joint')
        for child in joint.findall('child')
    }
    return sorted(links - child_links)


def _world_joint(robot: ET.Element) -> ET.Element:
    joint = robot.find("./joint[@name='panda_world_joint']")
    assert joint is not None
    assert joint.find('parent').attrib == {'link': 'world'}
    assert joint.find('child').attrib == {'link': 'panda_link0'}
    return joint


def _ros2_control_plugins(robot: ET.Element) -> dict[str, str]:
    plugins = {}
    for ros2_control in robot.findall('ros2_control'):
        plugin = ros2_control.find('./hardware/plugin')
        assert plugin is not None
        plugins[ros2_control.attrib['name']] = plugin.text
    return plugins


def _semantic_signature(srdf: ET.Element) -> Counter[tuple[str, tuple[tuple[str, str], ...]]]:
    signature = Counter()
    for element in srdf.iter():
        if element.tag in {'robot', 'virtual_joint'}:
            continue
        signature[(element.tag, tuple(sorted(element.attrib.items())))] += 1
    return signature


def _xml_signature(element: ET.Element) -> tuple:
    """Compare expanded xacro semantically while ignoring formatting whitespace."""
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or '').strip(),
        tuple(_xml_signature(child) for child in element),
    )
