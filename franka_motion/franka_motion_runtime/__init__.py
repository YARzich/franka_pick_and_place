"""
Robot-specific runtime helpers owned by the ``franka_motion`` ROS package.

Responsibility:
- Define a canonical namespace that cannot collide with ROSIDL-generated modules.
- Group Franka scenario and simulator runtime code.

Does not own:
- Robot-independent manipulation algorithms or backend contracts.
"""
