# Workflow Contract Tests

Responsibility: explain the robot-independent V2 workflow fixtures and the
behavior each test family proves.

## Neutral sequence lifecycle

The `transfer_sequence_action_core_*`, `transfer_sequence_session`, goal
transport, execution-policy, and reservation tests exercise the action-neutral
single-goal lifecycle without a planner, controller, TF listener, simulator, or
PlanningScene backend. They cover:

- provisional reservation and admission rollback;
- cancellation, pause, shutdown, and completion races;
- managed-worker startup, joining, and exactly-once terminal delivery;
- fail-closed validation of rich per-step execution evidence;
- release of the shared process-local reservation only after delivery and
  reusable worker state.

## V2 facade and resolved workflow

`test_transfer_object_sequence_v2_action_server.cpp` verifies the only public
transfer facade in this package. Domain tests exercise object-catalog lookup,
observation and grasp providers, strategy selection, scene-revision evidence,
typed feedback, and ordered cleanup through `ResolvedTransferWorkflow`.

These are CPU-only contract tests. Logical object dispositions are test
evidence and do not prove that a physical object moved.
