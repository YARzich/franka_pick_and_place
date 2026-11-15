"""
Validate the machine-readable Panda bindings against canonical robot resources.

Responsibility:
- Keep only live coordinator, health, Servo, V2 workflow, and diagnostic bindings.
- Cross-check profile frames, links, joints, groups, states, and controllers.

Does not own:
- Workflow timing, grasp sampling policy, or simulator runtime validation.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml

from config_test_utils import load_srdf, load_urdf


_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_PROFILE_PATH = (
    _PACKAGE_ROOT / "config" / "manipulation" / "robot_profile.yaml"
)

_PROFILE_KEYS = {
    "/**/planning_scene_coordinator": {
        "planning_frame",
        "backend_planning_frame",
    },
    "/**/system_health_monitor": {
        "base_frame",
        "end_effector_frame",
        "gripper_action",
        "required_joint_names",
        "required_controllers",
    },
    "/**/franka_moveit_servo_task_server": {
        "servo_to_pose.planning_frame",
        "servo_to_pose.task_frame",
        "servo_to_pose.servo_tip_frame",
        "servo_to_pose.joint_names",
        "servo_to_pose.arm_resource_id",
    },
    "/**/franka_transfer_object_sequence_server": {
        "robot.planning_frame",
        "robot.planning_group",
        "robot.end_effector_link",
        "robot.attachment_link",
        "robot.touch_links",
        "robot.open_width_m",
        "robot.allow_grasp_stall",
        "robot.home_joint_names",
        "robot.home_joint_positions",
        "robot.gripper_action",
    },
    "/**/franka_moveit_sequence_node": {
        "planning_group",
        "reference_frame",
    },
}

_POLICY_BINDINGS = (
    (
        "planning_scene_coordinator.yaml",
        "planning_scene_coordinator",
        _PROFILE_KEYS["/**/planning_scene_coordinator"],
    ),
    (
        "system_health.yaml",
        "system_health_monitor",
        _PROFILE_KEYS["/**/system_health_monitor"],
    ),
    (
        "servo_to_pose.yaml",
        "/**/franka_moveit_servo_task_server",
        _PROFILE_KEYS["/**/franka_moveit_servo_task_server"],
    ),
    (
        "transfer_object_sequence_v2.yaml",
        "franka_transfer_object_sequence_server",
        _PROFILE_KEYS["/**/franka_transfer_object_sequence_server"],
    ),
    (
        "moveit_backend.yaml",
        "/**/franka_transfer_object_sequence_server",
        _PROFILE_KEYS["/**/franka_transfer_object_sequence_server"],
    ),
    (
        "transfer_object_sequence_v2_deployment.yaml",
        "/**/franka_transfer_object_sequence_server",
        _PROFILE_KEYS["/**/franka_transfer_object_sequence_server"],
    ),
)


def test_profile_has_exact_live_node_scopes_without_yaml_indirection() -> None:
    """Reject hidden aliases, retired scopes, and unreviewed robot keys."""
    text = _PROFILE_PATH.read_text(encoding="utf-8")
    for event in yaml.parse(text):
        assert getattr(event, "anchor", None) is None

    profile = yaml.safe_load(text)
    assert set(profile) == set(_PROFILE_KEYS)
    for node_name, expected_keys in _PROFILE_KEYS.items():
        assert set(profile[node_name]) == {"ros__parameters"}
        assert set(profile[node_name]["ros__parameters"]) == expected_keys

    policy_directory = _PROFILE_PATH.parent
    for file_name, node_name, binding_keys in _POLICY_BINDINGS:
        policy = yaml.safe_load(
            (policy_directory / file_name).read_text(encoding="utf-8")
        )
        parameters = policy[node_name]["ros__parameters"]
        assert binding_keys.isdisjoint(parameters)


def test_profile_identifiers_resolve_in_robot_and_provider_resources() -> None:
    """Prove every retained Panda binding names an existing model entity."""
    profile = _load_profile()
    urdf = load_urdf()
    srdf = load_srdf()
    gazebo_urdf = _load_gazebo_urdf()
    isaac_urdf = _load_isaac_urdf()
    mtc = yaml.safe_load(
        (
            _PACKAGE_ROOT / "config" / "tasks" / "mtc_grasp_profiles.yaml"
        ).read_text(encoding="utf-8")
    )["grasp_profiles"]["panda_top_down_cube"]

    links = _names(urdf, "link")
    joints = _names(urdf, "joint")
    groups = {group.attrib["name"]: group for group in srdf.findall("group")}
    end_effectors = {
        end_effector.attrib["name"]: end_effector
        for end_effector in srdf.findall("end_effector")
    }

    coordinator = profile["planning_scene_coordinator"]
    health = profile["system_health_monitor"]
    servo = profile["franka_moveit_servo_task_server"]
    sequence = profile["franka_transfer_object_sequence_server"]
    diagnostic = profile["franka_moveit_sequence_node"]

    planning_frames = {
        coordinator["planning_frame"],
        servo["servo_to_pose.planning_frame"],
        sequence["robot.planning_frame"],
        diagnostic["reference_frame"],
    }
    assert planning_frames == {health["base_frame"]}
    assert planning_frames <= links
    assert coordinator["backend_planning_frame"] in _names(gazebo_urdf, "link")
    assert coordinator["backend_planning_frame"] in _names(isaac_urdf, "link")

    model_links = {
        health["end_effector_frame"],
        servo["servo_to_pose.task_frame"],
        servo["servo_to_pose.servo_tip_frame"],
        sequence["robot.end_effector_link"],
        sequence["robot.attachment_link"],
        mtc["ik_frame_link"],
        *sequence["robot.touch_links"],
    }
    assert model_links <= links

    assert set(health["required_joint_names"]) <= joints
    assert set(servo["servo_to_pose.joint_names"]) <= joints
    assert set(sequence["robot.home_joint_names"]) <= joints
    assert len(health["required_joint_names"]) == len(
        set(health["required_joint_names"])
    )
    assert len(health["required_controllers"]) == len(
        set(health["required_controllers"])
    )
    assert sequence["robot.home_joint_names"]
    assert len(sequence["robot.home_joint_names"]) == len(
        sequence["robot.home_joint_positions"]
    )
    assert len(sequence["robot.home_joint_names"]) == len(
        set(sequence["robot.home_joint_names"])
    )
    assert servo["servo_to_pose.joint_names"] == sequence[
        "robot.home_joint_names"
    ]

    planning_group = sequence["robot.planning_group"]
    assert planning_group in groups
    assert diagnostic["planning_group"] == planning_group
    assert mtc["arm_group"] == planning_group
    arm_chain = groups[planning_group].find("chain")
    assert arm_chain is not None
    assert arm_chain.attrib["tip_link"] == servo["servo_to_pose.servo_tip_frame"]

    assert mtc["eef"] in end_effectors
    assert mtc["ik_frame_link"] == sequence["robot.attachment_link"]
    assert health["end_effector_frame"] == sequence["robot.attachment_link"]
    assert health["gripper_action"] == sequence["robot.gripper_action"]
    gripper_config = yaml.safe_load(
        (
            _PACKAGE_ROOT / "config" / "control" /
            "panda_gripper_driver.yaml"
        ).read_text(encoding="utf-8")
    )
    assert gripper_config["panda_gripper_driver"]["ros__parameters"][
        "action_name"
    ] == sequence["robot.gripper_action"]

    expected_touch_links = set(sequence["robot.touch_links"])
    assert len(expected_touch_links) == len(sequence["robot.touch_links"])
    assert sequence["robot.attachment_link"] in expected_touch_links
    end_effector = end_effectors[mtc["eef"]]
    assert end_effector.attrib["parent_group"] == planning_group
    end_effector_group = groups[end_effector.attrib["group"]]
    assert expected_touch_links == {
        link.attrib["name"] for link in end_effector_group.findall("link")
    }
    for pose_name in (mtc["pregrasp_named_pose"], mtc["grasp_named_pose"]):
        assert srdf.find(
            f"./group_state[@group='{end_effector.attrib['group']}']"
            f"[@name='{pose_name}']"
        ) is not None

    ready = srdf.find("./group_state[@group='panda_arm'][@name='ready']")
    assert ready is not None
    ready_values = {
        joint.attrib["name"]: float(joint.attrib["value"])
        for joint in ready.findall("joint")
    }
    home_values = dict(
        zip(
            sequence["robot.home_joint_names"],
            sequence["robot.home_joint_positions"],
            strict=True,
        )
    )
    assert home_values == ready_values

    for controller_file in (
        "panda_ros2_control_gazebo.yaml",
        "panda_ros2_control_isaac.yaml",
    ):
        controller_config = yaml.safe_load(
            (
                _PACKAGE_ROOT / "config" / "control" / controller_file
            ).read_text(encoding="utf-8")
        )
        manager_parameters = controller_config["controller_manager"][
            "ros__parameters"
        ]
        assert set(health["required_controllers"]) <= manager_parameters.keys()


def _load_profile() -> dict[str, dict[str, object]]:
    """Return node parameters without ROS parameter-file scope wrappers."""
    profile = yaml.safe_load(_PROFILE_PATH.read_text(encoding="utf-8"))
    return {
        node_name.removeprefix("/**/"): node_config["ros__parameters"]
        for node_name, node_config in profile.items()
    }


def _load_gazebo_urdf():
    """Expand the canonical fixed-world Gazebo description without ROS nodes."""
    document = xacro.process_file(
        str(
            _PACKAGE_ROOT / "description" / "urdf" /
            "panda_gazebo.urdf.xacro"
        )
    )
    return ET.fromstring(document.toxml())


def _load_isaac_urdf():
    """Expand the canonical fixed-world Isaac description without ROS nodes."""
    document = xacro.process_file(
        str(
            _PACKAGE_ROOT / "description" / "urdf" /
            "panda_isaac.urdf.xacro"
        )
    )
    return ET.fromstring(document.toxml())


def _names(root, tag: str) -> set[str]:
    """Collect required name attributes from one URDF/SRDF element kind."""
    return {element.attrib["name"] for element in root.findall(tag)}
