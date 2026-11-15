// Responsibility: Stores one ServoToPose goal shared by executor callbacks and its owned worker.

#ifndef SERVO__SERVO_TO_POSE_ACTIVE_GOAL_HPP_
#define SERVO__SERVO_TO_POSE_ACTIVE_GOAL_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "manipulation/skill/resource_registry.hpp"
#include "manipulation_moveit/servo/servo_to_pose_action_server.hpp"

namespace manipulation::moveit
{

struct ServoToPoseActionServer::ActiveGoal
{
  using Clock = std::chrono::steady_clock;

  std::shared_ptr<GoalHandle> goal_handle;
  rclcpp_action::GoalUUID goal_id{};
  std::string episode_id;
  Clock::time_point started_at;
  Clock::time_point deadline;
  std::shared_ptr<ServoToPoseTaskControl> control;
  std::optional<manipulation::skill::ResourceRegistry::Lease> resource_lease;

  mutable std::mutex mutex;
  mutable std::mutex publish_mutex;
  ServoToPoseTaskFeedback feedback;
  std::optional<ServoToPoseTaskResult> result;
  bool worker_running{false};
  bool worker_finished{false};
  bool terminal_delivered{false};
  manipulation::task::TaskPhase last_recorded_phase{
    manipulation::task::TaskPhase::kIdle};
  bool last_recorded_paused{false};
};

}  // namespace manipulation::moveit

#endif  // SERVO__SERVO_TO_POSE_ACTIVE_GOAL_HPP_
