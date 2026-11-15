// Responsibility: Validates asynchronous joint/status feedback and builds fail-closed snapshots.

#include "moveit_servo_ros_adapter_impl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::moveit
{
namespace
{

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

bool is_fresh(
  const std::chrono::steady_clock::time_point received_at,
  const std::chrono::steady_clock::time_point now,
  const std::chrono::steady_clock::duration max_age) noexcept
{
  return now >= received_at && now - received_at <= max_age;
}

}  // namespace

bool MoveItServoRosAdapter::Impl::backend_ready() const noexcept
{
  try {
    if (!command_type_client_->service_is_ready() ||
      !pause_client_->service_is_ready() ||
      pose_publisher_->get_subscription_count() == 0U ||
      node_.now().nanoseconds() <= 0)
    {
      return false;
    }

    const auto now = SteadyClock::now();
    const std::lock_guard<std::mutex> lock(feedback_mutex_);
    const bool status_ready = status_feedback_.available &&
      is_known_moveit_servo_status(status_feedback_.code) &&
      (backend_state_ == manipulation::servo::ServoBackendState::kPaused ||
      is_fresh(status_feedback_.received_at, now, config_.status_max_age));
    return !ros_time_moved_backwards_ &&
           status_ready &&
           joint_feedback_.available && joint_feedback_.valid &&
           is_fresh(joint_feedback_.received_at, now, config_.joint_state_max_age);
  } catch (...) {
    return false;
  }
}

void MoveItServoRosAdapter::Impl::observe_ros_time_locked(const rclcpp::Time & now)
{
  const auto now_ns = now.nanoseconds();
  if (last_observed_ros_time_ns_ && now_ns < *last_observed_ros_time_ns_) {
    ros_time_moved_backwards_ = true;
  }
  last_observed_ros_time_ns_ = now_ns;
}

void MoveItServoRosAdapter::Impl::observe_message_stamp_locked(
  const builtin_interfaces::msg::Time & stamp,
  std::optional<std::int64_t> & previous_stamp)
{
  if (!manipulation::validation::is_normalized_nonzero_stamp(stamp)) {
    return;
  }
  const auto stamp_ns = stamp_nanoseconds(stamp);
  if (previous_stamp && stamp_ns < *previous_stamp) {
    ros_time_moved_backwards_ = true;
  }
  previous_stamp = stamp_ns;
}

void MoveItServoRosAdapter::Impl::reset_ros_time_epoch()
{
  const std::lock_guard<std::mutex> lock(feedback_mutex_);
  ros_time_moved_backwards_ = false;
  last_observed_ros_time_ns_ = node_.now().nanoseconds();
  last_joint_stamp_ns_.reset();
  last_tf_stamp_ns_.reset();
  joint_feedback_ = JointFeedback{};
  status_feedback_ = StatusFeedback{};
  pose_feedback_ = PoseFeedback{};
}

void MoveItServoRosAdapter::Impl::observe_joint_state(
  sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  const auto received_at = SteadyClock::now();
  const auto ros_now = node_.now();
  const auto stamp_ns = stamp_nanoseconds(message->header.stamp);
  const auto now_ns = ros_now.nanoseconds();
  const auto max_stamp_age_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    config_.joint_state_max_age).count();

  bool valid = manipulation::validation::is_normalized_nonzero_stamp(
    message->header.stamp) &&
    now_ns > 0 && stamp_ns <= now_ns && now_ns - stamp_ns <= max_stamp_age_ns &&
    message->name.size() == message->velocity.size();
  double max_abs_velocity = 0.0;
  if (valid) {
    std::unordered_map<std::string, double> velocities;
    velocities.reserve(message->name.size());
    for (std::size_t index = 0U; index < message->name.size(); ++index) {
      const double velocity = message->velocity[index];
      if (!std::isfinite(velocity) ||
        !velocities.emplace(message->name[index], velocity).second)
      {
        valid = false;
        break;
      }
    }
    if (valid) {
      for (const auto & required_joint : config_.joint_names) {
        const auto velocity = velocities.find(required_joint);
        if (velocity == velocities.end()) {
          valid = false;
          break;
        }
        max_abs_velocity = std::max(max_abs_velocity, std::abs(velocity->second));
      }
    }
  }

  const std::lock_guard<std::mutex> lock(feedback_mutex_);
  observe_ros_time_locked(ros_now);
  observe_message_stamp_locked(message->header.stamp, last_joint_stamp_ns_);
  const bool distinct = !joint_feedback_.available ||
    joint_feedback_.stamp != message->header.stamp;
  if (distinct) {
    ++joint_feedback_.sequence;
    joint_feedback_.available = true;
    joint_feedback_.valid = valid;
    joint_feedback_.max_abs_velocity = max_abs_velocity;
    joint_feedback_.stamp = message->header.stamp;
    joint_feedback_.received_at = received_at;
  }
}

void MoveItServoRosAdapter::Impl::observe_status(
  moveit_msgs::msg::ServoStatus::ConstSharedPtr message)
{
  const auto received_at = SteadyClock::now();
  const std::lock_guard<std::mutex> lock(feedback_mutex_);
  observe_ros_time_locked(node_.now());
  status_feedback_.available = true;
  status_feedback_.code = message->code;
  status_feedback_.message = message->message;
  status_feedback_.received_at = received_at;
}

MoveItServoRosObservation MoveItServoRosAdapter::Impl::update_and_snapshot_feedback(
  const std::optional<CurrentPoseMeasurement> & current_pose,
  const SteadyClock::time_point steady_now)
{
  const std::lock_guard<std::mutex> lock(feedback_mutex_);
  observe_ros_time_locked(node_.now());

  if (current_pose) {
    observe_message_stamp_locked(current_pose->stamp, last_tf_stamp_ns_);
    if (current_pose->time_discontinuity) {
      ros_time_moved_backwards_ = true;
    }
    const bool distinct = !pose_feedback_.available ||
      pose_feedback_.stamp != current_pose->stamp;
    if (distinct) {
      ++pose_feedback_.sequence;
      pose_feedback_.available = true;
      pose_feedback_.valid = current_pose->valid;
      pose_feedback_.position_error_m = current_pose->position_error_m;
      pose_feedback_.orientation_error_rad = current_pose->orientation_error_rad;
      pose_feedback_.stamp = current_pose->stamp;
      pose_feedback_.received_at = steady_now;
    }
    pose_feedback_.ros_fresh_at_capture = current_pose->fresh;
  }

  MoveItServoRosObservation observation;
  observation.backend_state = backend_state_;
  observation.ros_time_moved_backwards = ros_time_moved_backwards_;

  observation.pose_sample.sequence = pose_feedback_.sequence;
  observation.pose_sample.available = pose_feedback_.available;
  observation.pose_sample.valid = pose_feedback_.valid;
  observation.pose_sample.fresh = pose_feedback_.available &&
    pose_feedback_.ros_fresh_at_capture &&
    is_fresh(pose_feedback_.received_at, steady_now, config_.current_tf_max_age) &&
    !ros_time_moved_backwards_;
  observation.pose_sample.position_error_m = pose_feedback_.position_error_m;
  observation.pose_sample.orientation_error_rad = pose_feedback_.orientation_error_rad;

  observation.velocity_sample.sequence = joint_feedback_.sequence;
  observation.velocity_sample.available = joint_feedback_.available;
  observation.velocity_sample.valid = joint_feedback_.valid;
  observation.velocity_sample.fresh = joint_feedback_.available &&
    is_fresh(joint_feedback_.received_at, steady_now, config_.joint_state_max_age) &&
    !ros_time_moved_backwards_;
  observation.velocity_sample.max_abs_joint_velocity = joint_feedback_.max_abs_velocity;

  observation.status.available = status_feedback_.available;
  observation.status.valid = status_feedback_.available &&
    is_known_moveit_servo_status(status_feedback_.code);
  observation.status.fresh = status_feedback_.available &&
    is_fresh(status_feedback_.received_at, steady_now, config_.status_max_age) &&
    !ros_time_moved_backwards_;
  observation.status.code = status_feedback_.code;
  observation.status.message = status_feedback_.message;
  observation.safety_status = observation.status.valid && observation.status.fresh ?
    classify_moveit_servo_status(observation.status.code) :
    manipulation::servo::ServoSafetyStatus::kStopRequired;

  return observation;
}

}  // namespace manipulation::moveit
