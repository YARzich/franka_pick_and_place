// Responsibility: Validates ServoToPose task policy and synchronizes external control intent.

#include "manipulation_moveit/servo/servo_to_pose_task.hpp"

#include <exception>
#include <string>

#include "manipulation/servo/servo_to_pose_state_machine.hpp"

namespace manipulation::moveit
{

std::optional<std::string> validate_servo_to_pose_task_config(
  const ServoToPoseTaskConfig & config)
{
  using Duration = ServoToPoseTaskConfig::Duration;
  const struct
  {
    Duration value;
    const char * label;
  } durations[] = {
    {config.control_period, "control_period"},
    {config.incoming_command_timeout, "incoming_command_timeout"},
    {config.service_call_timeout, "service_call_timeout"},
    {config.target_prepare_timeout, "target_prepare_timeout"},
    {config.feedback_startup_timeout, "feedback_startup_timeout"},
    {config.observation_timeout, "observation_timeout"},
  };
  for (const auto & duration : durations) {
    if (duration.value <= Duration::zero()) {
      return std::string{duration.label} + " must be positive";
    }
  }
  if (config.control_period >= config.incoming_command_timeout) {
    return "control_period must be shorter than incoming_command_timeout";
  }
  if (config.observation_timeout > config.control_period) {
    return "observation_timeout must not exceed control_period";
  }
  try {
    (void)manipulation::servo::ServoToPoseStateMachine{config.state_machine};
  } catch (const std::exception & error) {
    return std::string{"invalid state-machine policy: "} + error.what();
  }
  return std::nullopt;
}

bool servo_to_pose_timeout_has_operational_budget(
  const ServoToPoseTaskConfig::Duration total_timeout,
  const ServoToPoseTaskConfig & config) noexcept
{
  return total_timeout > config.state_machine.halt_timeout &&
         total_timeout - config.state_machine.halt_timeout > config.control_period;
}

void ServoToPoseTaskControl::request_cancel()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    cancel_requested_ = true;
    ++revision_;
  }
  condition_.notify_all();
}

void ServoToPoseTaskControl::request_pause()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_requested_ = true;
    ++revision_;
  }
  condition_.notify_all();
}

void ServoToPoseTaskControl::resume()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pause_requested_ = false;
    ++revision_;
  }
  condition_.notify_all();
}

ServoToPoseTaskControl::Snapshot ServoToPoseTaskControl::snapshot() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return Snapshot{cancel_requested_, pause_requested_, revision_};
}

void ServoToPoseTaskControl::wait_until(
  const std::chrono::steady_clock::time_point deadline,
  const std::uint64_t revision)
{
  std::unique_lock<std::mutex> lock(mutex_);
  (void)condition_.wait_until(lock, deadline, [this, revision]() {
      return revision_ != revision;
    });
}

ServoToPoseTask::ServoToPoseTask(ServoToPosePort & port, ServoToPoseTaskConfig config)
: port_(port), config_(std::move(config))
{
  if (const auto error = validate_servo_to_pose_task_config(config_)) {
    throw std::invalid_argument("invalid ServoToPose task config: " + *error);
  }
}

const ServoToPoseTaskConfig & ServoToPoseTask::config() const noexcept
{
  return config_;
}

}  // namespace manipulation::moveit
