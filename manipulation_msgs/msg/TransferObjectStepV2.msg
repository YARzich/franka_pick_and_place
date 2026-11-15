# Responsibility: Describe one object-transfer intent resolved entirely by the server.
#
# The client identifies one physical object instance and its requested target.
# The server resolves the object profile, current stamped observation, geometry,
# grasp candidates, execution strategy, and backend configuration. Linear values
# use REP-103 metres.

# Stable physical-instance identifier. It must be a non-empty token without
# whitespace or '|'. The server resolves its profile from the configured catalog.
string object_id

uint8 TARGET_ABSOLUTE_POSE=0
uint8 TARGET_TRANSLATION_FROM_CURRENT_SOURCE=1
uint8 TARGET_INITIAL_SOURCE=2

# ABSOLUTE_POSE uses only target_object_pose. TRANSLATION_FROM_CURRENT_SOURCE
# adds target_translation to the freshly observed source position while
# preserving its orientation. INITIAL_SOURCE returns to the first successfully
# observed source pose for this object in the current goal. A first occurrence
# cannot use INITIAL_SOURCE.
uint8 target_mode

# Active only for TARGET_ABSOLUTE_POSE. The pose must carry a canonical frame_id,
# normalized non-zero stamp, finite position, and unit quaternion. In every other
# mode the complete message must equal a default-constructed message.
geometry_msgs/PoseStamped target_object_pose

# Active only for TARGET_TRANSLATION_FROM_CURRENT_SOURCE. The vector must carry
# the configured planning frame, a normalized non-zero stamp, and finite values.
# In every other mode the complete message must equal a default-constructed message.
geometry_msgs/Vector3Stamped target_translation

# Reaching the configured home state is a postcondition when true. Failure to
# return home keeps the step failed even if placement was already confirmed.
bool return_home

uint8 RECOVERY_NONE=0
uint8 RECOVERY_SAFE_STOP=1
uint8 RECOVERY_RETREAT=2
uint8 RECOVERY_RETREAT_AND_HOME=3

# Unsupported policies are rejected before observation, scene mutation, or any
# physical side effect. A server must never substitute a weaker policy.
uint8 recovery_policy
