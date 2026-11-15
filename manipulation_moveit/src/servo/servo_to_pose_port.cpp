// Responsibility: Forwards task port operations to the stable MoveIt Servo ROS adapter.

#include "manipulation_moveit/servo/servo_to_pose_port.hpp"

#include <exception>
#include <string>

namespace manipulation::moveit
{

MoveItServoTaskPort::MoveItServoTaskPort(MoveItServoRosAdapter & adapter) noexcept
: adapter_(adapter)
{
}

namespace
{

MoveItServoRosResult exception_result(const char * operation) noexcept
{
  try {
    throw;
  } catch (const std::exception & error) {
    return MoveItServoRosResult::failure(
      MoveItServoRosError::kInternalError,
      std::string{operation} + " threw: " + error.what());
  } catch (...) {
    return MoveItServoRosResult::failure(
      MoveItServoRosError::kInternalError,
      std::string{operation} + " threw an unknown exception");
  }
}

}  // namespace

bool MoveItServoTaskPort::backend_ready() const noexcept
{
  try {
    return adapter_.backend_ready();
  } catch (...) {
    return false;
  }
}

MoveItServoRosResult MoveItServoTaskPort::pause(const Deadline deadline) noexcept
{
  try {
    return adapter_.pause(deadline);
  } catch (...) {
    return exception_result("MoveIt Servo pause");
  }
}

MoveItServoRosResult MoveItServoTaskPort::configure_pose_mode(
  const Deadline deadline) noexcept
{
  try {
    return adapter_.configure_pose_mode(deadline);
  } catch (...) {
    return exception_result("MoveIt Servo command-mode configuration");
  }
}

MoveItServoPrepareTargetResult MoveItServoTaskPort::prepare_target(
  const geometry_msgs::msg::PoseStamped & target, const Deadline deadline) noexcept
{
  try {
    return adapter_.prepare_target(target, deadline);
  } catch (...) {
    return {exception_result("MoveIt Servo target preparation"), std::nullopt};
  }
}

MoveItServoRosResult MoveItServoTaskPort::unpause(const Deadline deadline) noexcept
{
  try {
    return adapter_.unpause(deadline);
  } catch (...) {
    return exception_result("MoveIt Servo resume");
  }
}

MoveItServoPublishResult MoveItServoTaskPort::publish_target() noexcept
{
  try {
    return adapter_.publish_target();
  } catch (...) {
    return {exception_result("MoveIt Servo target publication"), std::nullopt};
  }
}

MoveItServoObservationResult MoveItServoTaskPort::latest_observation(
  const Deadline deadline) noexcept
{
  try {
    return adapter_.latest_observation(deadline);
  } catch (...) {
    return {exception_result("MoveIt Servo observation"), {}};
  }
}

}  // namespace manipulation::moveit
