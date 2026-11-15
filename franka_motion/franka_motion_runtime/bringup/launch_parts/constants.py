"""
Shared launch constants for package, controller, topic and frame names.

Responsibility:
- Centralize stable names and simulator compatibility values used by launch modules.
- Keep cross-simulator topic/controller identifiers aligned.

Does not own:
- Scenario-specific poses, model paths or runtime parameter values.
- ROS node implementation constants outside launch assembly.
"""

from __future__ import annotations


SCENARIO_PACKAGE = "franka_motion"
BRINGUP_PACKAGE = SCENARIO_PACKAGE
CONTROL_PACKAGE = SCENARIO_PACKAGE
DESCRIPTION_PACKAGE = SCENARIO_PACKAGE
MOVEIT_CONFIG_PACKAGE = SCENARIO_PACKAGE
MTC_PACKAGE = SCENARIO_PACKAGE
STATIC_SCENE_PACKAGE = SCENARIO_PACKAGE

MANIPULATION_PACKAGE = "manipulation"
MOVEIT_BACKEND_PACKAGE = "manipulation_moveit"
PERCEPTION_PACKAGE = "manipulation_perception"
PERCEPTION_POSE_PACKAGE = PERCEPTION_PACKAGE
PERCEPTION_YOLO_PACKAGE = PERCEPTION_PACKAGE
GAZEBO_SIM_PACKAGE = SCENARIO_PACKAGE
ISAAC_SIM_PACKAGE = SCENARIO_PACKAGE
ROS_JAZZY_LIB_PATH = "/opt/ros/jazzy/lib"

ROBOT_NAME = "panda"
CONTROLLER_MANAGER = "/controller_manager"

JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
PANDA_ARM_CONTROLLER = "panda_arm_controller"
PANDA_HAND_CONTROLLER = "panda_hand_controller"

ISAAC_JOINT_COMMANDS_TOPIC = "/isaac_joint_commands"
ISAAC_JOINT_STATES_TOPIC = "/isaac_joint_states"

GAZEBO_GRIPPER_COMMAND_LIMITS = {
    "command_joint_min_position": 0.001,
    "command_joint_max_position": 0.039,
}
