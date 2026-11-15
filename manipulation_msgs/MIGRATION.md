<!-- Responsibility: Record the completed V1/backend-action removal and V2 migration. -->

# Transfer workflow V2 migration

`TransferObjectSequenceV2` was introduced additively in
`manipulation_msgs` 0.2.0. Repository consumers migrated while the old types
remained available. Version 0.3.0 completed the reviewed breaking migration:
the V1 transfer surface and the backend-specific `CuRoboPlanPose` action were
removed without a compatibility bridge.

The current public transfer goal contains only `TransferObjectStepV2[]`. Object
geometry, source observation, grasp generation, strategy and planner selection
are server/deployment configuration. `ServoToPose`, scene-coordination, health
and pause interfaces were retained unchanged.

## Type mapping

| Removed type | Replacement or disposition |
| --- | --- |
| `StaticBoxTransferStep` | `TransferObjectStepV2`; geometry/source/grasp move to server providers |
| `PerceptionResolvedTransferStep` | `TransferObjectStepV2`; observation mode is server configuration |
| `TransferObjectSequence` | `TransferObjectSequenceV2` |
| `PerceptionResolvedTransferSequence` | `TransferObjectSequenceV2` |
| `TransferObject` | No ROS replacement; the V2 strategy calls internal planning, execution, gripper and scene ports |
| `CuRoboPlanPose` | No ROS replacement; optional cuRobo is a MoveIt `PlannerManager` selected by backend profile |

No removed endpoint was rebound to a different wire type in place.

## Endpoint mapping

| Removed endpoint | V2 replacement or disposition |
| --- | --- |
| `/transfer_object_sequence` | `/transfer_object_sequence_v2` |
| `/perception_resolved_transfer_sequence` | `/transfer_object_sequence_v2` |
| `/set_transfer_object_sequence_paused` | `/set_transfer_object_sequence_v2_paused` |
| `/set_perception_resolved_transfer_sequence_paused` | `/set_transfer_object_sequence_v2_paused` |
| `/franka_moveit_transfer_object_server/transfer_object` | No ROS replacement; internal strategy/backend ports |
| `/franka_moveit_transfer_object_server/set_task_paused` | No replacement; the V2 lifecycle owns pause |
| `/plan_curobo_pose` | No ROS replacement; select the cuRobo planning profile before startup |

The shared `SetTaskPaused` service type remains public because the V2 transfer
server and `ServoToPose` use it. Only obsolete endpoint instances were removed.

## Consumer and removal gates

The repository/workspace inventory is preserved in
[`docs/transfer_v1_removal_inventory.md`](docs/transfer_v1_removal_inventory.md).
The removal was applied only after package owners confirmed that:

1. configured and detected clients use `TransferObjectSequenceV2`;
2. the V2 server invokes its strategy directly, with no inner transfer action;
3. obsolete installed headers, CMake targets, executables, launch aliases and
   runtime configurations have a deletion or V2 disposition;
4. cuRobo planning is reached through the same internal arm-planning port and
   canonical MoveIt `PlannerManager` path;
5. source searches outside historical evidence and negative architecture tests
   find no removed wire-type or endpoint consumer.

No known external consumer was registered for this candidate reference
application, so the migration intentionally has no bridge or deprecation
runtime. An untracked external checkout must update and rebuild all consumers
before using 0.3.0.

The package install step removes only the generated files belonging to the six
retired types. This matters for an incremental workspace upgrade: rosidl does
not otherwise delete obsolete `.idl`, headers, or private Python modules from
an existing install prefix.

## Contract evidence

`docs/public_wire_contract.v1.json` is the byte-exact pre-migration archive. It
is installed as historical evidence but does not generate ROS types.
`docs/public_wire_contract.v2.json` is the exact current source contract. Tests
compare every source and generated Python type with V2, verify unchanged
retained contracts against V1, and assert that removed types remain archive
only.
