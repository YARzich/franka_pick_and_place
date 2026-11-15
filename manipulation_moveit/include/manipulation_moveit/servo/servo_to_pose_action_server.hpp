// Responsibility: Defines the single-goal ROS action facade for the reusable ServoToPose task.

#ifndef MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_ACTION_SERVER_HPP_
#define MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_ACTION_SERVER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

#include "manipulation/workflow/skill_runtime.hpp"
#include "manipulation_msgs/action/servo_to_pose.hpp"
#include "manipulation_msgs/srv/set_task_paused.hpp"
#include "manipulation_moveit/servo/servo_to_pose_task.hpp"

namespace manipulation::moveit
{

/** Validated action, timeout, resource, and task policy for one server. */
struct ServoToPoseActionServerConfig
{
  using Duration = std::chrono::steady_clock::duration;

  std::string action_name{"~/servo_to_pose"};
  std::string pause_service_name{"~/servo_to_pose/set_task_paused"};
  Duration heartbeat_period{std::chrono::milliseconds{25}};
  Duration default_timeout{std::chrono::seconds{30}};
  ServoToPoseTaskConfig task;
  std::shared_ptr<const manipulation::workflow::SkillRuntime> skill_runtime;
};

/** Read-only lifecycle view for health monitoring and deterministic tests. */
struct ServoToPoseActionServerSnapshot
{
  bool goal_reserved{false};
  bool active{false};
  bool worker_running{false};
  bool result_pending{false};
  bool resource_held{false};
  bool resource_quarantined{false};
  manipulation::task::TaskPhase phase{manipulation::task::TaskPhase::kIdle};
  bool paused{false};
  bool pause_pending{false};
};

/**
 * Thin ROS transport owner around ServoToPoseTask.
 *
 * The server borrows its node and port; both must outlive it. Production must
 * pass a SkillRuntime backed by the same ResourceRegistry as every other arm
 * command skill in the host. The task worker is owned and joined; no callback
 * spins an executor and no thread is detached.
 */
class ServoToPoseActionServer final
{
public:
  using Action = manipulation_msgs::action::ServoToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using PauseService = manipulation_msgs::srv::SetTaskPaused;

  ServoToPoseActionServer(
    rclcpp::Node & node,
    ServoToPosePort & port,
    ServoToPoseActionServerConfig config);
  ~ServoToPoseActionServer();

  ServoToPoseActionServer(const ServoToPoseActionServer &) = delete;
  ServoToPoseActionServer & operator=(const ServoToPoseActionServer &) = delete;
  ServoToPoseActionServer(ServoToPoseActionServer &&) = delete;
  ServoToPoseActionServer & operator=(ServoToPoseActionServer &&) = delete;

  [[nodiscard]] ServoToPoseActionServerSnapshot snapshot() const;

private:
  struct ActiveGoal;

  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & goal_id,
    std::shared_ptr<const Action::Goal> goal);
  [[nodiscard]] rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_pause_request(
    const std::shared_ptr<PauseService::Request> request,
    std::shared_ptr<PauseService::Response> response);

  void run_task(const std::shared_ptr<ActiveGoal> & active);
  void accept_feedback(
    const std::shared_ptr<ActiveGoal> & active,
    const ServoToPoseTaskFeedback & feedback);
  void heartbeat();
  void publish_feedback(const std::shared_ptr<ActiveGoal> & active);
  void deliver_result(const std::shared_ptr<ActiveGoal> & active);
  void release_completed_goal();
  void record_event(
    const std::shared_ptr<ActiveGoal> & active,
    manipulation::recording::EpisodeLifecycleMarker marker,
    const ServoToPoseTaskResult * result = nullptr) const noexcept;

  [[nodiscard]] bool pause_request_matches(
    const unique_identifier_msgs::msg::UUID & requested,
    const std::shared_ptr<ActiveGoal> & active) const noexcept;

  const ServoToPoseActionServerConfig config_;
  const rclcpp::Logger logger_;
  ServoToPosePort & port_;
  ServoToPoseTask task_;

  mutable std::mutex goal_mutex_;
  bool goal_reserved_{false};
  bool cancel_before_accept_{false};
  rclcpp_action::GoalUUID reserved_goal_id_{};
  std::shared_ptr<ActiveGoal> active_goal_;
  std::thread worker_;
  bool backend_ready_announced_{false};

  // A failed stop proof makes subsequent arm commands unsafe. Retaining the
  // original lease avoids any release/reacquire gap and clears only when this
  // process-local action server is destroyed during host restart.
  std::optional<manipulation::skill::ResourceRegistry::Lease>
  quarantined_resource_lease_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Service<PauseService>::SharedPtr pause_service_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__SERVO__SERVO_TO_POSE_ACTION_SERVER_HPP_
