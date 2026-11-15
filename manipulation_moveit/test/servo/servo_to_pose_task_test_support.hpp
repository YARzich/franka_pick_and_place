// Responsibility: Provides a deterministic in-memory Servo port for task and action tests.

#ifndef SERVO__SERVO_TO_POSE_TASK_TEST_SUPPORT_HPP_
#define SERVO__SERVO_TO_POSE_TASK_TEST_SUPPORT_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "manipulation_moveit/servo/servo_to_pose_port.hpp"
#include "manipulation_moveit/servo/servo_to_pose_task.hpp"

namespace manipulation::moveit::test
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

inline geometry_msgs::msg::PoseStamped valid_target()
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "world";
  target.header.stamp.sec = 1;
  target.pose.orientation.w = 1.0;
  return target;
}

inline ServoToPoseTaskConfig fast_task_config()
{
  ServoToPoseTaskConfig config;
  config.control_period = 1ms;
  config.incoming_command_timeout = 20ms;
  config.service_call_timeout = 5ms;
  config.target_prepare_timeout = 5ms;
  config.feedback_startup_timeout = 10ms;
  config.observation_timeout = 1ms;
  config.state_machine.position_tolerance_m = 0.01;
  config.state_machine.orientation_tolerance_rad = 0.02;
  config.state_machine.stopped_velocity_tolerance = 0.03;
  config.state_machine.required_target_samples = 2U;
  config.state_machine.required_stopped_samples = 2U;
  config.state_machine.goal_timeout = 1s;
  config.state_machine.halt_timeout = 15ms;
  return config;
}

class FakeServoToPosePort final : public ServoToPosePort
{
public:
  enum class Mode
  {
    kSuccess,
    kNeverReach,
    kStaleAfterPublish,
    kSafetyStopAfterPublish,
    kThrowOnceAfterPublish,
    kFinalPoseOutsideTolerance,
    kFinalPoseStale,
    kFinalPoseInvalid,
  };

  explicit FakeServoToPosePort(const Mode mode = Mode::kSuccess)
  : mode_(mode)
  {
  }

  bool backend_ready() const noexcept override
  {
    return backend_ready_flag.load();
  }

  MoveItServoRosResult pause(Deadline) noexcept override
  {
    ++pause_calls;
    backend_state_.store(manipulation::servo::ServoBackendState::kPaused);
    return MoveItServoRosResult::success();
  }

  MoveItServoRosResult configure_pose_mode(Deadline) noexcept override
  {
    ++configure_calls;
    return MoveItServoRosResult::success();
  }

  MoveItServoPrepareTargetResult prepare_target(
    const geometry_msgs::msg::PoseStamped & target, const Deadline deadline) noexcept override
  {
    ++prepare_calls;
    if (block_prepare_until_deadline.load()) {
      std::this_thread::sleep_until(deadline);
      return {
        MoveItServoRosResult::failure(
          MoveItServoRosError::kTimeout, "scripted preparation deadline"),
        std::nullopt};
    }
    if (fail_prepare.load()) {
      return {
        MoveItServoRosResult::failure(
          MoveItServoRosError::kTimeout, "scripted exact-stamp TF timeout"),
        std::nullopt};
    }
    MoveItServoPreparedTarget prepared;
    prepared.task_goal = target;
    prepared.servo_tip_goal = target;
    return {MoveItServoRosResult::success(), prepared};
  }

  MoveItServoRosResult unpause(Deadline) noexcept override
  {
    ++unpause_calls;
    backend_state_.store(manipulation::servo::ServoBackendState::kRunning);
    return MoveItServoRosResult::success();
  }

  MoveItServoPublishResult publish_target() noexcept override
  {
    ++publish_calls;
    return {MoveItServoRosResult::success(), valid_target()};
  }

  MoveItServoObservationResult latest_observation(Deadline) override
  {
    if (publish_calls.load() > 0U && mode_ == Mode::kThrowOnceAfterPublish &&
      throw_count.exchange(1U) == 0U)
    {
      throw std::runtime_error("scripted observation exception after tracking started");
    }
    const auto sequence = ++observation_sequence_;
    MoveItServoRosObservation observation;
    observation.backend_state = backend_state_.load();
    observation.safety_status = manipulation::servo::ServoSafetyStatus::kNominal;
    const bool ready = sequence > readiness_after_observations.load();
    observation.status.available = ready;
    observation.status.fresh = ready;
    observation.status.valid = ready;
    observation.status.code = 0;
    if (suppress_status_feedback.load() ||
      (suppress_status_while_paused.load() &&
      backend_state_.load() == manipulation::servo::ServoBackendState::kPaused &&
      pause_calls.load() >= 2U))
    {
      // MoveIt Servo 2.12.x does not publish ServoStatus while paused. Keep
      // the last known status but expose it as stale, just like the ROS adapter.
      observation.status.available = true;
      observation.status.fresh = false;
      observation.status.valid = true;
      observation.safety_status = manipulation::servo::ServoSafetyStatus::kStopRequired;
    }

    observation.pose_sample.available = true;
    observation.pose_sample.fresh = true;
    observation.pose_sample.valid = true;
    observation.pose_sample.sequence = sequence;
    observation.pose_sample.position_error_m =
      mode_ == Mode::kNeverReach ? 0.5 : 0.001;
    observation.pose_sample.orientation_error_rad =
      mode_ == Mode::kNeverReach ? 0.5 : 0.001;

    observation.velocity_sample.available = true;
    observation.velocity_sample.fresh = true;
    observation.velocity_sample.valid = true;
    observation.velocity_sample.sequence = sequence;
    observation.velocity_sample.max_abs_joint_velocity = allow_low_velocity.load() ? 0.0 : 0.2;

    const bool final_pose_observation = publish_calls.load() > 0U && pause_calls.load() >= 2U;
    if (final_pose_observation && mode_ == Mode::kFinalPoseOutsideTolerance) {
      observation.pose_sample.position_error_m = 0.1;
    }
    if (final_pose_observation && mode_ == Mode::kFinalPoseStale) {
      observation.pose_sample.fresh = false;
    }
    if (final_pose_observation && mode_ == Mode::kFinalPoseInvalid) {
      observation.pose_sample.valid = false;
    }

    if (publish_calls.load() > 0 && mode_ == Mode::kStaleAfterPublish) {
      observation.pose_sample.fresh = false;
    }
    if (publish_calls.load() > 0 && mode_ == Mode::kSafetyStopAfterPublish) {
      observation.safety_status = manipulation::servo::ServoSafetyStatus::kStopRequired;
    }
    return {MoveItServoRosResult::success(), observation};
  }

  std::atomic_uint32_t pause_calls{0U};
  std::atomic_uint32_t configure_calls{0U};
  std::atomic_uint32_t prepare_calls{0U};
  std::atomic_uint32_t unpause_calls{0U};
  std::atomic_uint32_t publish_calls{0U};
  std::atomic_uint32_t throw_count{0U};
  std::atomic_bool allow_low_velocity{true};
  std::atomic_bool fail_prepare{false};
  std::atomic_bool block_prepare_until_deadline{false};
  std::atomic_bool backend_ready_flag{true};
  std::atomic_bool suppress_status_feedback{false};
  std::atomic_bool suppress_status_while_paused{false};
  std::atomic_uint64_t readiness_after_observations{0U};

private:
  const Mode mode_;
  std::atomic<manipulation::servo::ServoBackendState> backend_state_{
    manipulation::servo::ServoBackendState::kPaused};
  std::atomic_uint64_t observation_sequence_{0U};
};

template<typename Predicate>
bool wait_until(Predicate predicate, const std::chrono::milliseconds timeout = 100ms)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

}  // namespace manipulation::moveit::test

#endif  // SERVO__SERVO_TO_POSE_TASK_TEST_SUPPORT_HPP_
