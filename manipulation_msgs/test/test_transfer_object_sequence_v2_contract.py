"""
Freeze the additive TransferObjectSequenceV2 source and generated contract.

Responsibility:
- Assert the exact V2 intent fields and forbid a whole-goal timeout.
- Freeze typed result and waiting-feedback fields and numeric constants.
- Confirm generated Python classes expose the same field order and constants.
"""

from pathlib import Path
from typing import Any

from manipulation_msgs.action import TransferObjectSequenceV2
from manipulation_msgs.msg import TransferObjectStepV2
from rosidl_adapter.parser import parse_action_file
from rosidl_adapter.parser import parse_message_file


PACKAGE_NAME = 'manipulation_msgs'
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _field_signature(field: Any) -> tuple[str | None, str, bool, int | None, bool]:
    """Return the exact authored type shape needed by this focused contract."""
    type_spec = field.type
    return (
        type_spec.pkg_name,
        type_spec.type,
        type_spec.is_array,
        type_spec.array_size,
        type_spec.is_upper_bound,
    )


def _fields(specification: Any) -> list[tuple[str, tuple[Any, ...]]]:
    """Return ordered source field names and type signatures."""
    return [
        (field.name, _field_signature(field)) for field in specification.fields
    ]


def _constants(specification: Any) -> list[tuple[str, str, Any]]:
    """Return ordered source constants with their authored primitive types."""
    return [
        (constant.name, constant.type, constant.value)
        for constant in specification.constants
    ]


def test_transfer_object_step_v2_source_contract() -> None:
    """Freeze the server-resolved intent and its target/recovery enums."""
    specification = parse_message_file(
        PACKAGE_NAME, PACKAGE_ROOT / 'msg' / 'TransferObjectStepV2.msg'
    )
    assert _fields(specification) == [
        ('object_id', (None, 'string', False, None, False)),
        ('target_mode', (None, 'uint8', False, None, False)),
        (
            'target_object_pose',
            ('geometry_msgs', 'PoseStamped', False, None, False),
        ),
        (
            'target_translation',
            ('geometry_msgs', 'Vector3Stamped', False, None, False),
        ),
        ('return_home', (None, 'bool', False, None, False)),
        ('recovery_policy', (None, 'uint8', False, None, False)),
    ]
    assert _constants(specification) == [
        ('TARGET_ABSOLUTE_POSE', 'uint8', 0),
        ('TARGET_TRANSLATION_FROM_CURRENT_SOURCE', 'uint8', 1),
        ('TARGET_INITIAL_SOURCE', 'uint8', 2),
        ('RECOVERY_NONE', 'uint8', 0),
        ('RECOVERY_SAFE_STOP', 'uint8', 1),
        ('RECOVERY_RETREAT', 'uint8', 2),
        ('RECOVERY_RETREAT_AND_HOME', 'uint8', 3),
    ]


def test_transfer_object_sequence_v2_source_contract() -> None:
    """Freeze goal/result/feedback and prove the V2 goal has only steps."""
    specification = parse_action_file(
        PACKAGE_NAME, PACKAGE_ROOT / 'action' / 'TransferObjectSequenceV2.action'
    )
    assert _fields(specification.goal) == [
        (
            'steps',
            (PACKAGE_NAME, 'TransferObjectStepV2', True, None, False),
        )
    ]
    assert _constants(specification.goal) == []
    assert _fields(specification.result) == [
        ('result', (PACKAGE_NAME, 'ManipulationTaskResult', False, None, False)),
        ('completed_step_count', (None, 'uint32', False, None, False)),
        ('failed_step_index', (None, 'int64', False, None, False)),
        (
            'step_results',
            (PACKAGE_NAME, 'ManipulationTaskResult', True, None, False),
        ),
    ]
    assert _constants(specification.result) == [('NO_FAILED_STEP', 'int64', -1)]
    assert _fields(specification.feedback) == [
        ('sequence_step_index', (None, 'uint32', False, None, False)),
        ('sequence_step_count', (None, 'uint32', False, None, False)),
        ('overall_progress', (None, 'float32', False, None, False)),
        ('active_object_id', (None, 'string', False, None, False)),
        ('current_state', (PACKAGE_NAME, 'ManipulationTaskState', False, None, False)),
        ('waiting_reason', (None, 'uint8', False, None, False)),
    ]
    assert _constants(specification.feedback) == [
        ('WAITING_NONE', 'uint8', 0),
        ('WAITING_OBSERVATION', 'uint8', 1),
        ('WAITING_TRANSFORM', 'uint8', 2),
        ('WAITING_GRASP_GENERATION', 'uint8', 3),
        ('WAITING_SCENE_READINESS', 'uint8', 4),
        ('WAITING_HEALTH_READINESS', 'uint8', 5),
        ('WAITING_PLANNING', 'uint8', 6),
        ('WAITING_ROS_ENDPOINT', 'uint8', 7),
    ]


def test_generated_v2_contract_matches_authored_field_order() -> None:
    """Confirm rosidl generated the reviewed V2 source surface."""
    assert list(TransferObjectStepV2.get_fields_and_field_types()) == [
        'object_id',
        'target_mode',
        'target_object_pose',
        'target_translation',
        'return_home',
        'recovery_policy',
    ]
    assert list(TransferObjectSequenceV2.Goal.get_fields_and_field_types()) == [
        'steps'
    ]
    assert list(TransferObjectSequenceV2.Result.get_fields_and_field_types()) == [
        'result',
        'completed_step_count',
        'failed_step_index',
        'step_results',
    ]
    assert list(TransferObjectSequenceV2.Feedback.get_fields_and_field_types()) == [
        'sequence_step_index',
        'sequence_step_count',
        'overall_progress',
        'active_object_id',
        'current_state',
        'waiting_reason',
    ]
    assert TransferObjectStepV2.TARGET_INITIAL_SOURCE == 2
    assert TransferObjectStepV2.RECOVERY_RETREAT_AND_HOME == 3
    assert TransferObjectSequenceV2.Result.NO_FAILED_STEP == -1
    assert TransferObjectSequenceV2.Feedback.WAITING_ROS_ENDPOINT == 7
