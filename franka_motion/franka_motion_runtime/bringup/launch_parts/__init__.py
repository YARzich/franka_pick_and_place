"""
Reusable launch factories owned by the Franka integration layer.

Responsibility:
- Assemble simulator, control, MoveIt and perception actions without running them.

Does not own:
- Backend algorithms, robot descriptions, controller YAML or simulator assets.
"""
