"""
Reusable launch building blocks for franka_motion.

Responsibility:
- Provide the import namespace for shared launch helper modules.
- Keep simulator, motion and perception launch helpers grouped under one package.

Does not own:
- Runtime ROS node implementation.
- Scenario-specific launch entrypoint assembly.
"""
