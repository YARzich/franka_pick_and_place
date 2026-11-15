"""
Compatibility alias for the canonical detection_arguments.py launch helper.

Responsibility:
- Preserve the historical franka_motion_parts import path.
- Expose the exact canonical module object so monkeypatching and caches agree.

Does not own:
- Launch implementation, state or resource-path policy.
"""

import sys

from franka_motion_runtime.bringup.launch_parts.camera import detection_arguments as _canonical_module

sys.modules[__name__] = _canonical_module
