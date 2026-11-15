// Responsibility: Names stable robot-independent capabilities used by skill descriptors.

#ifndef MANIPULATION__SKILL__STANDARD_CAPABILITIES_HPP_
#define MANIPULATION__SKILL__STANDARD_CAPABILITIES_HPP_

#include <string_view>

namespace manipulation::skill::capability
{

inline constexpr std::string_view kPlanExecutePose{"motion/plan_execute_pose"};
inline constexpr std::string_view kCartesianPath{"motion/cartesian_path"};
inline constexpr std::string_view kServoPose{"motion/servo_pose"};
inline constexpr std::string_view kGripperWidthCommand{"gripper/width_command"};
inline constexpr std::string_view kCoordinatedSceneMutation{
  "scene/coordinated_mutation"};
inline constexpr std::string_view kExactStampTransform{"tf/exact_stamp"};
inline constexpr std::string_view kRigidObjectWorldState{"world/rigid_object"};

}  // namespace manipulation::skill::capability

#endif  // MANIPULATION__SKILL__STANDARD_CAPABILITIES_HPP_
