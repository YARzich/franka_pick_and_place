# Responsibility: Represent backend-independent manipulation progress and pause state.
#
# A pause request is cooperative. While an in-flight controller/MoveIt call or
# bounded stop confirmation is in progress, pause_pending is true. paused
# becomes true at the next safe checkpoint. phase continues to describe the
# physical workflow phase.

uint8 IDLE=0
uint8 VALIDATING=1
uint8 WAITING_FOR_RESOURCES=2
uint8 PLANNING=3
uint8 APPROACHING=4
uint8 GRASPING=5
uint8 ATTACHING=6
uint8 LIFTING=7
uint8 TRANSFERRING=8
uint8 PLACING=9
uint8 RELEASING=10
uint8 DETACHING=11
uint8 RETREATING=12
uint8 RECOVERING=13
uint8 COMPLETED=14
uint8 CANCELED=15
uint8 FAILED=16

# A reactive controller is actively updating commands toward its target.
uint8 TRACKING=17

# Current physical workflow phase. Constants are append-only; clients must
# tolerate a newer phase while still using progress and detail for display.
uint8 phase

# Monotonic workflow progress in [0.0, 1.0]. Recovery may hold progress but
# must not decrease it.
float32 progress

# Zero-based index of the current safe-checkpoint step and total step count.
uint16 step_index
uint16 step_count

# Retry counts the current phase's bounded retries; recovery_count counts
# recovery attempts started over the complete goal.
uint16 retry_count
uint16 recovery_count
bool paused
bool pause_pending

# Goal-relative steady-clock duration, not ROS time.
builtin_interfaces/Duration elapsed

# Human-readable detail for operators. Clients must branch on phase/error
# codes, not parse this string.
string detail
