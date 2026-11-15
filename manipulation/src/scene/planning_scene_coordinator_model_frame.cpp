// Responsibility: Discovers the backend planning frame and verifies it against the fixed robot
//   root.

#include "planning_scene_coordinator_node.hpp"

#include <mutex>
#include <string>

#include "manipulation/scene/robot_description_frame.hpp"

namespace manipulation::scene
{

bool PlanningSceneCoordinatorNode::backend_model_frame_ready() const
{
  std::lock_guard<std::mutex> lock(model_frame_mutex_);
  return backend_model_frame_verified_;
}

void PlanningSceneCoordinatorNode::handle_robot_description(
  const std_msgs::msg::String::SharedPtr message)
{
  const auto parsed = parse_fixed_root_frame(message ? message->data : std::string{});
  const bool verified = parsed.valid && parsed.frame_id == backend_planning_frame_;
  std::string status_message;
  if (!parsed.valid) {
    status_message = "cannot verify MoveIt model frame: " + parsed.message;
  } else if (!verified) {
    status_message =
      "configured backend_planning_frame '" + backend_planning_frame_ +
      "' differs from fixed-root URDF frame '" + parsed.frame_id + "'";
  } else {
    status_message =
      "backend_planning_frame matches fixed-root URDF model frame";
  }

  {
    std::lock_guard<std::mutex> lock(model_frame_mutex_);
    backend_model_frame_verified_ = verified;
    observed_model_frame_ = parsed.frame_id;
    model_frame_verification_message_ = status_message;
  }

  if (!verified) {
    require_reconciliation(RegistryError::kBackendFailed, status_message);
    RCLCPP_ERROR(get_logger(), "%s", status_message.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "%s", status_message.c_str());
}

}  // namespace manipulation::scene
