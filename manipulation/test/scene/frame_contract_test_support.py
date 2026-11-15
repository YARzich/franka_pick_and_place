"""
Provide message builders for PlanningScene frame launch tests.

Responsibility: Construct leases and scene diffs with explicit object, frame, and operation data.
"""

from geometry_msgs.msg import Pose
from manipulation_msgs.srv import ApplyPlanningSceneDiff
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


APPLY_BACKEND = '/frame_contract/apply_planning_scene'
READBACK_BACKEND = '/frame_contract/get_planning_scene'
COORDINATOR = '/frame_contract_coordinator'


def robot_description(root_frame: str) -> String:
    """Build a minimal fixed-root robot description."""
    return String(data=(
        f"<robot name='test'><link name='{root_frame}'/>"
        "<link name='robot_base'/><joint name='root' type='fixed'>"
        f"<parent link='{root_frame}'/><child link='robot_base'/>"
        '</joint></robot>'
    ))


def scene_diff(x: float = 0.0) -> PlanningScene:
    """Build one explicit cube ADD in the public planning frame."""
    scene = PlanningScene()
    scene.is_diff = True
    object_message = CollisionObject()
    object_message.id = 'cube'
    object_message.header.frame_id = 'robot_base'
    object_message.operation = CollisionObject.ADD
    object_message.pose.position.x = x
    object_message.pose.orientation.w = 1.0
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.05, 0.05, 0.05]
    object_message.primitives.append(box)
    local_pose = Pose()
    local_pose.orientation.w = 1.0
    object_message.primitive_poses.append(local_pose)
    scene.world.collision_objects.append(object_message)
    return scene


def apply_request(
    lease_id: str,
    operation_id: str,
    revision: int,
    x: float = 0.0,
) -> ApplyPlanningSceneDiff.Request:
    """Build one coordinator request with exact resource coverage."""
    request = ApplyPlanningSceneDiff.Request()
    request.lease_id = lease_id
    request.operation_id = operation_id
    request.expected_revision = revision
    request.resource_keys = ['world:cube']
    request.scene_diff = scene_diff(x)
    return request
