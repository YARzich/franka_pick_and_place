# Responsibility: Represent a typed backend-independent manipulation result.

uint16 NONE=0
uint16 CANCELED=1
uint16 INVALID_GOAL=100
uint16 RESOURCE_UNAVAILABLE=101
uint16 ROBOT_STATE_STALE=102
uint16 CONTROLLER_UNAVAILABLE=103
uint16 TF_ERROR=104
uint16 PERCEPTION_TIMEOUT=105
uint16 PLANNING_FAILED=106
uint16 EXECUTION_FAILED=107
uint16 GRIPPER_FAILED=108
uint16 SCENE_CONFLICT=109
uint16 SCENE_FAILED=110
uint16 TIMEOUT=111
uint16 RECOVERY_FAILED=112
uint16 INTERNAL_ERROR=255

uint8 DISPOSITION_UNKNOWN=0
uint8 SAFE_IDLE=1
uint8 OBJECT_AT_SOURCE=2
uint8 HOLDING_OBJECT=3
uint8 OBJECT_AT_TARGET=4
uint8 SCENE_UNCERTAIN=5

# True only when the requested postconditions were confirmed. A false value
# does not imply that the robot is idle or that the object was released.
bool success

# Machine-readable terminal reason. Values are append-only; consumers must
# tolerate unknown future values and must not parse error_message.
uint16 error_code

# Human-readable context. Automation must use error_code.
string error_message

# Best-known physical/scene disposition after execution and bounded recovery.
# This field remains meaningful on failure and cancel.
uint8 final_disposition

# Number of recovery attempts actually started by this goal.
uint16 recovery_count

# Goal-relative elapsed time measured with a steady clock. It is not a ROS
# timestamp and remains monotonic across /clock pauses and resets.
builtin_interfaces/Duration elapsed
