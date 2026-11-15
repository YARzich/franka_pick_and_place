// Responsibility: Encodes ROS action UUIDs as stable episode and logical owner identities.

#ifndef WORKFLOW__DETAIL__ACTION_GOAL_IDENTITY_HPP_
#define WORKFLOW__DETAIL__ACTION_GOAL_IDENTITY_HPP_

#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"

namespace manipulation::workflow::detail
{

inline std::string goal_episode_id(const rclcpp_action::GoalUUID & goal_id)
{
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0');
  for (const std::uint8_t byte : goal_id) {
    encoded << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return encoded.str();
}

inline std::string goal_resource_owner_id(const rclcpp_action::GoalUUID & goal_id)
{
  return "goal/id_" + goal_episode_id(goal_id);
}

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__ACTION_GOAL_IDENTITY_HPP_
