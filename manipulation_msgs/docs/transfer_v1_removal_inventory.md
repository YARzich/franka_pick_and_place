<!-- Responsibility: Preserve the source-consumer evidence for removed wire contracts. -->

# Removed transfer and cuRobo wire-contract inventory

Final source audit: 2026-07-23.

Scope: the `ros2_examples` repository and sibling source trees under
`/home/yaroslav/ros2_ws/src`. Generated `build`, `install` and `log` trees,
negative architecture assertions, and the historical
`public_wire_contract.v1.json` archive are not live consumers.

## Removed contracts and endpoints

- `manipulation_msgs/action/TransferObject`
- `manipulation_msgs/action/TransferObjectSequence`
- `manipulation_msgs/action/PerceptionResolvedTransferSequence`
- `manipulation_msgs/action/CuRoboPlanPose`
- `manipulation_msgs/msg/StaticBoxTransferStep`
- `manipulation_msgs/msg/PerceptionResolvedTransferStep`
- `/transfer_object_sequence`
- `/perception_resolved_transfer_sequence`
- `/set_transfer_object_sequence_paused`
- `/set_perception_resolved_transfer_sequence_paused`
- `/franka_moveit_transfer_object_server/transfer_object`
- `/franka_moveit_transfer_object_server/set_task_paused`
- `/plan_curobo_pose`

## Repository consumers and disposition

All live consumers belonged to the same `manipulation` family:

- `manipulation`: V1 action facades, clients, validators, static-box adapters,
  executables, exported targets and tests were removed; the package now exposes
  one V2 workflow lifecycle and internal strategy/provider ports.
- `manipulation_moveit`: the old transfer action host and V1-specific backend
  surface were removed; the V2 strategy uses the common planning, execution,
  gripper, health and scene ports directly.
- `franka_motion`: configured and detected clients moved to the V2 endpoint;
  V1 hosts, compatibility entrypoints, launch wiring and configuration were
  removed or rewritten.
- `manipulation_curobo` and `franka_motion`: the direct pose-planning action,
  bridge, server, launch aliases and demos were removed. Optional cuRobo now
  enters through the canonical MoveIt planner plugin and common planning port.

The final source audit found no positive reference to a removed type or
endpoint outside migration/history documents. Explicit negative architecture
tests retain some names so accidental reintroduction continues to fail.

## External gate

The original audit found no reference in another `ros2_examples` family or
sibling source tree. No external consumer is registered for this candidate
reference application. The removal therefore has no parallel compatibility
period or bridge. `MIGRATION.md` records replacements, and the immutable V1
snapshot preserves the old semantic wire shape for review evidence.
