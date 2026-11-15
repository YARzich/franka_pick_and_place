"""
Compatibility package for the canonical Isaac simulator runtime.

Responsibility:
    Keep historical `franka_motion.scripts.isaac_runtime` imports resolvable
    while the implementation lives in
    ``franka_motion_runtime.simulation.isaac.runtime``.

Does not own:
    Isaac runtime behavior or simulator resources.

New code should import
:mod:`franka_motion_runtime.simulation.isaac.runtime` directly. These modules
remain installed so existing scripts keep their historical imports.
"""
