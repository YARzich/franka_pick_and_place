# Responsibility: Represent a typed terminal result for backend-independent robot motion.

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

# True only when the requested motion postconditions were confirmed. A false
# value does not imply that the robot is stopped.
bool success

# Machine-readable terminal reason. Values are append-only; consumers must
# tolerate unknown future values and must not parse error_message.
uint16 error_code

# Human-readable context. Automation must use error_code.
string error_message

# True only when the server observed its configured consecutive count of
# distinct, fresh robot-state samples and every controlled joint reported a
# finite absolute velocity within the configured stop threshold. A controller
# acknowledgement, a sent zero command, or elapsed settling time alone is not
# stop confirmation. False means that rest was not proven; it does not by itself
# prove that the robot is moving.
bool stop_confirmed

# Number of recovery attempts actually started by this goal.
uint16 recovery_count

# Goal-relative elapsed time measured with a steady clock. It is not a ROS
# timestamp and remains monotonic across /clock pauses and resets.
builtin_interfaces/Duration elapsed
