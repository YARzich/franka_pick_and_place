"""
YOLO detector backend for the stamped perception contract.

Responsibility:
    Mark the package boundary for YOLO inference, result normalization, stamped
    ROS output, and optional debug-control adapters.

Does not own:
    Stereo depth, 6D pose estimation, or motion planning.
"""
