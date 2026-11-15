// Responsibility: Implements interruptible observation acquisition without semantic wait limits.

#include "workflow_v2/observation_providers.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

using manipulation::task::TaskErrorCode;
using manipulation::workflow::ObjectObservation;
using manipulation::workflow::ObservationProviderResult;
using manipulation::workflow::TransferExecutionControlResult;
using manipulation::workflow::TransferExecutionContext;
using manipulation::workflow::TransferWaitingReason;

constexpr auto kInterruptibleWaitSlice = std::chrono::milliseconds{50};

std::string correlation_id(
  const std::string & object_id, const builtin_interfaces::msg::Time & stamp)
{
  return object_id + "@" + std::to_string(stamp.sec) + "." +
         std::to_string(stamp.nanosec);
}

ObservationProviderResult canceled_result()
{
  return ObservationProviderResult::failure(
    TaskErrorCode::kCanceled, "observation wait was canceled or shut down");
}

manipulation::task::TaskStateSnapshot waiting_feedback(
  const std::string & object_id, const std::string & detail)
{
  manipulation::task::TaskStateSnapshot feedback;
  feedback.phase = manipulation::task::TaskPhase::kWaitingForResources;
  feedback.detail = object_id + ": " + detail;
  return feedback;
}

std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

}  // namespace

ConfiguredObservationProvider::ConfiguredObservationProvider(
  ConfiguredObservationPreset observations,
  StampSource stamp_source)
: observations_(std::move(observations)), stamp_source_(std::move(stamp_source))
{
  if (observations_.empty() || !stamp_source_) {
    throw std::invalid_argument(
        "configured observation provider requires poses and a stamp source");
  }
  for (const auto & [object_id, observation] : observations_) {
    geometry_msgs::msg::PoseStamped candidate;
    candidate.header.frame_id = observation.frame_id;
    candidate.header.stamp.sec = 1;
    candidate.pose = observation.pose;
    if (object_id.empty() || !manipulation::validation::is_valid_pose_stamped(candidate) ||
      (observation.confidence &&
      (!std::isfinite(*observation.confidence) || *observation.confidence < 0.0 ||
      *observation.confidence > 1.0)))
    {
      throw std::invalid_argument("configured observation preset contains an invalid pose");
    }
  }
}

ObservationProviderResult ConfiguredObservationProvider::observe(
  const manipulation::workflow::ObjectInstance & instance,
  const manipulation::workflow::ObjectProfile & profile,
  TransferExecutionContext & context)
{
  const auto request_generation = stop_generation_.load(std::memory_order_acquire);
  if (instance.profile_id() != profile.profile_id()) {
    return ObservationProviderResult::failure(
      TaskErrorCode::kInvalidGoal, "configured observation received a mismatched object profile");
  }
  if (stop_generation_.load(std::memory_order_acquire) != request_generation ||
    context.checkpoint() == TransferExecutionControlResult::kStopped)
  {
    return canceled_result();
  }
  const auto found = observations_.find(instance.object_id());
  if (found == observations_.cend()) {
    return ObservationProviderResult::failure(
      TaskErrorCode::kInvalidGoal,
      "configured observation preset has no object '" + instance.object_id() + "'");
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = found->second.frame_id;
  pose.pose = found->second.pose;
  while (true) {
    pose.header.stamp = stamp_source_();
    if (manipulation::validation::is_normalized_nonzero_stamp(pose.header.stamp)) {
      break;
    }
    context.report_waiting(
      TransferWaitingReason::kObservation,
      waiting_feedback(instance.object_id(), "waiting for a non-zero observation stamp"));
    if (stop_generation_.load(std::memory_order_acquire) != request_generation ||
      context.wait_for(kInterruptibleWaitSlice) == TransferExecutionControlResult::kStopped)
    {
      return canceled_result();
    }
  }
  context.report_waiting(
    TransferWaitingReason::kNone,
    waiting_feedback(instance.object_id(), "configured observation stamped"));
  try {
    return ObservationProviderResult::success(ObjectObservation::create(
      instance.object_id(), pose, correlation_id(instance.object_id(), pose.header.stamp),
      found->second.confidence));
  } catch (const std::invalid_argument & error) {
    return ObservationProviderResult::failure(TaskErrorCode::kInternalError, error.what());
  }
}

void ConfiguredObservationProvider::request_stop() noexcept
{
  stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
}

DetectedObservationProvider::DetectedObservationProvider(
  DetectedPoseBuffer & pose_buffer,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  DetectedObservationProviderConfig config,
  StampSource stamp_source)
: pose_buffer_(pose_buffer), tf_buffer_(std::move(tf_buffer)), config_(std::move(config)),
  stamp_source_(std::move(stamp_source))
{
  if (!tf_buffer_ || !stamp_source_ || config_.object_id.empty() ||
    !std::isfinite(config_.max_observation_age_sec) || config_.max_observation_age_sec <= 0.0 ||
    !manipulation::validation::is_canonical_relative_frame_id(config_.target_frame) ||
    (config_.minimum_position_z && !std::isfinite(*config_.minimum_position_z)))
  {
    throw std::invalid_argument("detected observation provider configuration is invalid");
  }
}

bool DetectedObservationProvider::stopped(
  const TransferExecutionContext & context,
  const std::uint64_t request_generation) const noexcept
{
  return stop_generation_.load(std::memory_order_acquire) != request_generation ||
         context.stop_requested();
}

ObservationProviderResult DetectedObservationProvider::observe(
  const manipulation::workflow::ObjectInstance & instance,
  const manipulation::workflow::ObjectProfile & profile,
  TransferExecutionContext & context)
{
  const auto request_generation = stop_generation_.load(std::memory_order_acquire);
  if (instance.object_id() != config_.object_id || instance.profile_id() != profile.profile_id()) {
    return ObservationProviderResult::failure(
      TaskErrorCode::kInvalidGoal, "detected observation provider does not own this object");
  }
  auto not_before = DetectedPoseBuffer::SteadyClock::now();
  std::optional<geometry_msgs::msg::PoseStamped> detected;
  while (!stopped(context, request_generation)) {
    if (context.checkpoint() == TransferExecutionControlResult::kStopped) {
      return canceled_result();
    }
    bool reset = false;
    const auto now = observe_clock(reset);
    if (reset) {
      detected.reset();
      not_before = DetectedPoseBuffer::SteadyClock::now();
    }
    if (detected && !fresh_stamp(detected->header.stamp, now)) {
      detected.reset();
      not_before = DetectedPoseBuffer::SteadyClock::now();
    }
    if (!detected) {
      context.report_waiting(
        TransferWaitingReason::kObservation,
        waiting_feedback(instance.object_id(), "waiting for a fresh non-replayed observation"));
      detected = pose_buffer_.waitForFreshCubePose(
        kInterruptibleWaitSlice, not_before,
        [this, &context, request_generation]() {
          return !stopped(context, request_generation);
        },
        [this, now](const auto & pose) {
          return manipulation::validation::is_valid_pose_stamped(pose) &&
                 fresh_stamp(pose.header.stamp, now);
        });
      // Re-read ROS time after receiving a sample before using even an identity TF.
      continue;
    }

    geometry_msgs::msg::PoseStamped transformed;
    if (detected->header.frame_id == config_.target_frame) {
      transformed = *detected;
    } else {
      context.report_waiting(
        TransferWaitingReason::kTransform,
        waiting_feedback(instance.object_id(), "waiting for exact-stamp transform"));
      try {
        transformed = tf_buffer_->transform(
          *detected, config_.target_frame, tf2::durationFromSec(0.0));
        transformed.header.frame_id = config_.target_frame;
        transformed.header.stamp = detected->header.stamp;
      } catch (const tf2::TransformException &) {
        if (context.wait_for(kInterruptibleWaitSlice) == TransferExecutionControlResult::kStopped) {
          return canceled_result();
        }
        continue;
      }
    }

    // A slow TF lookup may consume the sample budget or cross a simulated-time reset.
    const auto completed_at = observe_clock(reset);
    if (reset || !fresh_stamp(transformed.header.stamp, completed_at)) {
      detected.reset();
      not_before = DetectedPoseBuffer::SteadyClock::now();
      continue;
    }
    if (stopped(context, request_generation)) {
      return canceled_result();
    }
    if (config_.minimum_position_z &&
      transformed.pose.position.z < *config_.minimum_position_z)
    {
      transformed.pose.position.z = *config_.minimum_position_z;
    }
    try {
      auto observation = ObjectObservation::create(
        instance.object_id(), transformed,
        correlation_id(instance.object_id(), transformed.header.stamp));
      consumed_stamp_ = stamp_nanoseconds(transformed.header.stamp);
      context.report_waiting(
        TransferWaitingReason::kNone,
        waiting_feedback(instance.object_id(), "fresh exact-stamp observation ready"));
      return ObservationProviderResult::success(std::move(observation));
    } catch (const std::invalid_argument & error) {
      return ObservationProviderResult::failure(TaskErrorCode::kTfError, error.what());
    }
  }
  return canceled_result();
}

bool DetectedObservationProvider::fresh_stamp(
  const builtin_interfaces::msg::Time & stamp, const std::int64_t now) const noexcept
{
  if (now <= 0 || !manipulation::validation::is_normalized_nonzero_stamp(stamp)) {
    return false;
  }
  const auto sample = stamp_nanoseconds(stamp);
  return sample <= now && (!consumed_stamp_ || sample > *consumed_stamp_) &&
         static_cast<double>(now - sample) / 1.0e9 <= config_.max_observation_age_sec;
}

std::int64_t DetectedObservationProvider::observe_clock(bool & reset)
{
  const auto stamp = stamp_source_();
  reset = false;
  if (!manipulation::validation::is_normalized_stamp(stamp)) {
    return 0;
  }
  const auto now = stamp_nanoseconds(stamp);
  reset = last_ros_now_ && now < *last_ros_now_;
  if (reset) {
    consumed_stamp_.reset();
  }
  last_ros_now_ = now;
  return now;
}

void DetectedObservationProvider::request_stop() noexcept
{
  stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
  pose_buffer_.notifyWaiters();
}

}  // namespace franka_motion::workflow_v2
