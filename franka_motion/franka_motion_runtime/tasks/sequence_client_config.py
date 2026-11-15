"""
Load the V2 action endpoint and indexed transfer intents.

Responsibility:
- Declare the stable ROS parameter surface of the sequence request client.
- Validate the endpoint and reconstruct pure V2 intent.

Does not own:
- Action transport, process signals, server policy, or clock-domain selection.
"""

from __future__ import annotations

from dataclasses import dataclass

from manipulation_msgs.msg import TransferObjectStepV2
from rclpy.node import Node

from franka_motion_runtime.tasks.model import TransferObjectIntent
from franka_motion_runtime.tasks.request_parsing import transfer_steps_from_parameters


@dataclass(frozen=True)
class SequenceClientConfig:
    """Hold the client-owned endpoint separately from action intent."""

    action_name: str

    def __post_init__(self) -> None:
        """Reject an endpoint that cannot identify the canonical action."""
        if not self.action_name:
            raise ValueError("action.name must not be empty")


def load_sequence_client_config(
    node: Node,
    *,
    default_action_name: str = "/transfer_object_sequence_v2",
) -> SequenceClientConfig:
    """Declare only the single V2 endpoint; waits are not client policy."""
    action_name = node.declare_parameter(
        "action.name",
        default_action_name,
    ).value
    return SequenceClientConfig(
        action_name=str(action_name),
    )


def _declare_target_parameters(
    node: Node,
    *,
    prefix: str,
    target_mode: object,
) -> dict[str, object]:
    """Declare only fields active for the selected target expression."""
    defaults: dict[str, object] = {}
    if target_mode == TransferObjectStepV2.TARGET_ABSOLUTE_POSE:
        defaults = {
            f"{prefix}.target_object_pose.frame_id": "",
            f"{prefix}.target_object_pose.position": [0.0, 0.0, 0.0],
            f"{prefix}.target_object_pose.orientation_xyzw": [
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        }
    elif target_mode == TransferObjectStepV2.TARGET_TRANSLATION_FROM_CURRENT_SOURCE:
        defaults = {
            f"{prefix}.target_translation.frame_id": "",
            f"{prefix}.target_translation.vector": [0.0, 0.0, 0.0],
        }
    return {
        name: node.declare_parameter(name, default).value
        for name, default in defaults.items()
    }


def _declare_step_parameters(node: Node) -> dict[str, object]:
    """Read launch-generated V2 intent from declared parameter overrides."""
    count = node.declare_parameter("steps.count", 0).value
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("steps.count must be a positive integer")
    mapping: dict[str, object] = {"steps.count": count}
    for index in range(count):
        prefix = f"steps.{index}"
        common_defaults: dict[str, object] = {
            f"{prefix}.object_id": "",
            f"{prefix}.target_mode": TransferObjectStepV2.TARGET_ABSOLUTE_POSE,
            f"{prefix}.return_home": True,
            f"{prefix}.recovery_policy": TransferObjectStepV2.RECOVERY_SAFE_STOP,
        }
        common = {
            name: node.declare_parameter(name, default).value
            for name, default in common_defaults.items()
        }
        mapping.update(common)
        mapping.update(
            _declare_target_parameters(
                node,
                prefix=prefix,
                target_mode=common[f"{prefix}.target_mode"],
            )
        )
    return mapping


def load_sequence_client_inputs(
    node: Node,
) -> tuple[SequenceClientConfig, tuple[TransferObjectIntent, ...]]:
    """Load and validate local policy and all intent before action creation."""
    config = load_sequence_client_config(node)
    steps = transfer_steps_from_parameters(_declare_step_parameters(node))
    return config, steps
