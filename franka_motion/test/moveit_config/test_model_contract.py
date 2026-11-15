"""
Cross-check the fixed-base SRDF against the expanded canonical URDF.

Responsibility: Verify groups, named states, limits, and collision pairs reference valid entities.
"""

from config_test_utils import ARM_JOINTS, FINGER_JOINTS, load_srdf, load_urdf


def _names(root, tag: str) -> set[str]:
    return {element.attrib['name'] for element in root.findall(tag)}


def test_urdf_wrapper_exports_expected_fixed_base_model():
    """The generic wrapper must contain geometry but no backend control plugin."""
    urdf = load_urdf()
    links = _names(urdf, 'link')
    joints = _names(urdf, 'joint')

    assert urdf.attrib['name'] == 'panda'
    assert {'panda_link0', 'panda_link8', 'panda_hand'} <= links
    assert set(ARM_JOINTS + FINGER_JOINTS) <= joints
    assert urdf.find('ros2_control') is None

    parented_links = {
        joint.find('child').attrib['link']
        for joint in urdf.findall('joint')
        if joint.find('child') is not None
    }
    assert 'panda_link0' not in parented_links


def test_srdf_groups_keep_public_names_and_fixed_base_chain():
    """Keep stable group names and the REP-103 fixed-base chain contract."""
    srdf = load_srdf()
    groups = {group.attrib['name']: group for group in srdf.findall('group')}

    assert set(groups) == {'panda_arm', 'hand', 'panda_arm_hand'}
    chain = groups['panda_arm'].find('chain')
    assert chain is not None
    assert chain.attrib == {
        'base_link': 'panda_link0',
        'tip_link': 'panda_link8',
    }
    assert srdf.find('virtual_joint') is None

    hand_joints = {joint.attrib['name'] for joint in groups['hand'].findall('joint')}
    passive = {
        joint.attrib['name'] for joint in groups['hand'].findall('passive_joint')
    }
    assert hand_joints == {'panda_finger_joint1'}
    assert passive == {'panda_finger_joint2'}

    subgroups = {
        group.attrib['name'] for group in groups['panda_arm_hand'].findall('group')
    }
    assert subgroups == {'panda_arm', 'hand'}


def test_srdf_references_only_existing_urdf_entities():
    """Every semantic link and joint reference must resolve in the URDF."""
    urdf = load_urdf()
    srdf = load_srdf()
    urdf_links = _names(urdf, 'link')
    urdf_joints = _names(urdf, 'joint')

    for group in srdf.findall('group'):
        for link in group.findall('link'):
            assert link.attrib['name'] in urdf_links
        for tag in ('joint', 'passive_joint'):
            for joint in group.findall(tag):
                assert joint.attrib['name'] in urdf_joints

    for state in srdf.findall('group_state'):
        for joint in state.findall('joint'):
            assert joint.attrib['name'] in urdf_joints

    end_effector = srdf.find("end_effector[@name='hand']")
    assert end_effector is not None
    assert end_effector.attrib == {
        'group': 'hand',
        'name': 'hand',
        'parent_group': 'panda_arm',
        'parent_link': 'panda_link8',
    }


def test_named_states_are_complete_and_inside_position_limits():
    """Reject incomplete or physically invalid named SRDF states."""
    srdf = load_srdf()
    states = {
        (state.attrib['group'], state.attrib['name']): state
        for state in srdf.findall('group_state')
    }
    assert set(states) == {
        ('panda_arm', 'ready'),
        ('panda_arm', 'extended'),
        ('panda_arm', 'transport'),
        ('hand', 'open'),
        ('hand', 'close'),
    }

    position_limits = {
        joint.attrib['name']: (
            float(joint.find('limit').attrib['lower']),
            float(joint.find('limit').attrib['upper']),
        )
        for joint in load_urdf().findall('joint')
        if joint.find('limit') is not None
        and 'lower' in joint.find('limit').attrib
        and 'upper' in joint.find('limit').attrib
    }
    for (group, _), state in states.items():
        expected = ARM_JOINTS if group == 'panda_arm' else FINGER_JOINTS
        values = {
            joint.attrib['name']: float(joint.attrib['value'])
            for joint in state.findall('joint')
        }
        assert set(values) == set(expected)
        for joint, value in values.items():
            lower, upper = position_limits[joint]
            assert lower <= value <= upper


def test_disabled_collision_pairs_are_valid_and_unique():
    """The allowed-collision matrix must not contain stale or duplicate pairs."""
    urdf_links = _names(load_urdf(), 'link')
    pairs = []

    for disabled in load_srdf().findall('disable_collisions'):
        link1 = disabled.attrib['link1']
        link2 = disabled.attrib['link2']
        assert link1 in urdf_links
        assert link2 in urdf_links
        assert link1 != link2
        assert disabled.attrib['reason'] in {'Adjacent', 'Default', 'Never'}
        pairs.append(tuple(sorted((link1, link2))))

    assert len(pairs) == len(set(pairs))
