// Responsibility: Evaluate numeric joint quiescence and clock epochs without ROS resources.

#ifndef QUIESCENCE_POLICY_HPP_
#define QUIESCENCE_POLICY_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace manipulation::moveit::detail
{

constexpr std::size_t kRequiredLowVelocitySamples = 2U;
constexpr std::size_t kRequiredPositionStabilitySamples = 5U;
constexpr double kTerminalPositionTolerance = 0.01;
constexpr double kTerminalVelocityTolerance = 0.01;
constexpr double kPositionStabilityTolerance = 0.001;
constexpr auto kPositionStabilityWindow = std::chrono::milliseconds{100};
constexpr auto kJointStateMaxAge = std::chrono::milliseconds{250};

struct JointTarget
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
};

struct RequiredJointSample
{
  std::vector<double> positions;
  std::optional<std::vector<double>> velocities;
  std::int64_t stamp_nanoseconds{0};
};

class QuiescenceClock final
{
public:
  explicit QuiescenceClock(const std::int64_t barrier)
  : barrier_(barrier), previous_(barrier) {}

  bool observe(const std::int64_t now, std::string & error)
  {
    if (now <= 0 || now < previous_) {
      error = "ROS clock moved backwards during joint-state quiescence; now_ns=" +
        std::to_string(now) + " previous_now_ns=" + std::to_string(previous_) +
        " epoch_barrier_ns=" + std::to_string(barrier_);
      return false;
    }
    previous_ = now;
    return true;
  }

private:
  std::int64_t barrier_;
  std::int64_t previous_;
};

enum class QuiescenceEvidence
{
  kPending,
  kReady,
  kInvalid,
};

enum class QuiescenceObservation
{
  kNone,
  kStale,
  kNonMonotonic,
  kOutsideTarget,
  kAtTarget,
};

struct TargetErrorEvidence
{
  std::string joint_name;
  double actual{0.0};
  double target{0.0};
  double absolute_error{0.0};
};

struct VelocityEvidence
{
  std::string joint_name;
  double signed_velocity{0.0};
  double absolute_velocity{0.0};
};

inline VelocityEvidence maximum_velocity_evidence(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & velocities)
{
  VelocityEvidence result{
    joint_names.front(), velocities.front(), std::abs(velocities.front())};
  for (std::size_t index = 1U; index < velocities.size(); ++index) {
    const auto absolute_velocity = std::abs(velocities[index]);
    if (absolute_velocity > result.absolute_velocity) {
      result = VelocityEvidence{
        joint_names[index], velocities[index], absolute_velocity};
    }
  }
  return result;
}

class QuiescenceTracker final
{
public:
  QuiescenceEvidence observe(
    const RequiredJointSample & sample,
    const JointTarget & target,
    const std::int64_t barrier_stamp_nanoseconds,
    const std::int64_t ros_now_nanoseconds,
    const std::chrono::steady_clock::time_point received_at,
    std::string & error)
  {
    error.clear();
    const auto finite = [](const std::vector<double> & values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
                   return std::isfinite(value);
        });
      };
    if (target.names.empty() || target.names.size() != target.positions.size() ||
      sample.positions.size() != target.positions.size() ||
      (sample.velocities && sample.velocities->size() != target.names.size()) ||
      !finite(target.positions) || !finite(sample.positions) ||
      (sample.velocities && !finite(*sample.velocities)) || sample.stamp_nanoseconds <= 0)
    {
      error = "quiescence requires complete finite joint evidence and a positive stamp";
      return QuiescenceEvidence::kInvalid;
    }
    ++examined_samples_;
    last_sample_stamp_nanoseconds_ = sample.stamp_nanoseconds;
    last_ros_now_nanoseconds_ = ros_now_nanoseconds;
    epoch_barrier_nanoseconds_ = barrier_stamp_nanoseconds;
    if (sample.stamp_nanoseconds > ros_now_nanoseconds) {
      error = "JointState timestamp is ahead of the ROS clock; stamp_ns=" +
        std::to_string(sample.stamp_nanoseconds) +
        " now_ns=" + std::to_string(ros_now_nanoseconds) +
        " epoch_barrier_ns=" + std::to_string(barrier_stamp_nanoseconds);
      return QuiescenceEvidence::kInvalid;
    }
    const auto max_age_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kJointStateMaxAge).count();
    if (sample.stamp_nanoseconds < barrier_stamp_nanoseconds ||
      ros_now_nanoseconds - sample.stamp_nanoseconds > max_age_ns)
    {
      reset_evidence();
      last_observation_ = QuiescenceObservation::kStale;
      previous_stamp_nanoseconds_ =
        std::max(previous_stamp_nanoseconds_, sample.stamp_nanoseconds);
      return QuiescenceEvidence::kPending;
    }
    if (sample.stamp_nanoseconds <= previous_stamp_nanoseconds_) {
      reset_evidence();
      last_observation_ = QuiescenceObservation::kNonMonotonic;
      return QuiescenceEvidence::kPending;
    }
    ++fresh_samples_;
    previous_stamp_nanoseconds_ = sample.stamp_nanoseconds;

    TargetErrorEvidence target_error{
      target.names.front(), sample.positions.front(), target.positions.front(),
      std::abs(sample.positions.front() - target.positions.front())};
    for (std::size_t index = 1U; index < target.positions.size(); ++index) {
      const auto absolute_error =
        std::abs(sample.positions[index] - target.positions[index]);
      if (absolute_error > target_error.absolute_error) {
        target_error = TargetErrorEvidence{
          target.names[index], sample.positions[index], target.positions[index],
          absolute_error};
      }
    }
    last_target_error_evidence_ = target_error;
    last_velocity_evidence_.reset();
    if (sample.velocities) {
      last_velocity_evidence_ =
        maximum_velocity_evidence(target.names, *sample.velocities);
    }

    if (target_error.absolute_error > kTerminalPositionTolerance) {
      reset_evidence();
      last_observation_ = QuiescenceObservation::kOutsideTarget;
      return QuiescenceEvidence::kPending;
    }
    last_observation_ = QuiescenceObservation::kAtTarget;

    if (sample.velocities) {
      const bool calm = std::all_of(
        sample.velocities->cbegin(), sample.velocities->cend(),
        [](const double velocity) {
          return std::abs(velocity) <= kTerminalVelocityTolerance;
        });
      low_velocity_samples_ = calm ? low_velocity_samples_ + 1U : 0U;
    } else {
      low_velocity_samples_ = 0U;
    }

    observe_position_window(sample.positions, received_at);
    const bool position_window_ready =
      position_window_samples_ >= kRequiredPositionStabilitySamples &&
      position_window_start_ &&
      latest_position_sample_ - *position_window_start_ >= kPositionStabilityWindow;
    return (low_velocity_samples_ >= kRequiredLowVelocitySamples ||
           position_window_ready) ?
           QuiescenceEvidence::kReady : QuiescenceEvidence::kPending;
  }

  [[nodiscard]] std::string diagnostic(const JointTarget & target) const
  {
    const double sample_age_ms =
      static_cast<double>(last_ros_now_nanoseconds_ - last_sample_stamp_nanoseconds_) / 1.0e6;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "examined_samples=" << examined_samples_
           << " fresh_samples=" << fresh_samples_
           << " last_stamp_ns=" << last_sample_stamp_nanoseconds_
           << " last_ros_now_ns=" << last_ros_now_nanoseconds_
           << " epoch_barrier_ns=" << epoch_barrier_nanoseconds_
           << " sample_age_ms=" << sample_age_ms
           << " max_sample_age_ms=" << kJointStateMaxAge.count()
           << " last_observation=" << observation_label(last_observation_);
    if (last_target_error_evidence_) {
      stream << " max_target_error=" << last_target_error_evidence_->absolute_error
             << " max_target_error_joint=" << last_target_error_evidence_->joint_name
             << " max_target_actual=" << last_target_error_evidence_->actual
             << " max_target=" << last_target_error_evidence_->target;
    } else {
      stream << " max_target_error=unavailable"
             << " max_target_error_joint=unavailable"
             << " max_target_actual=unavailable"
             << " max_target=unavailable";
    }
    if (last_velocity_evidence_) {
      stream << " max_abs_velocity=" << last_velocity_evidence_->absolute_velocity
             << " max_abs_velocity_joint=" << last_velocity_evidence_->joint_name
             << " max_abs_velocity_signed=" << last_velocity_evidence_->signed_velocity;
    } else {
      stream << " max_abs_velocity=unavailable"
             << " max_abs_velocity_joint=unavailable"
             << " max_abs_velocity_signed=unavailable";
    }
    if (!target.velocities.empty()) {
      const auto terminal_velocity =
        maximum_velocity_evidence(target.names, target.velocities);
      stream << " max_terminal_waypoint_abs_velocity="
             << terminal_velocity.absolute_velocity
             << " max_terminal_waypoint_velocity_joint="
             << terminal_velocity.joint_name
             << " max_terminal_waypoint_velocity_signed="
             << terminal_velocity.signed_velocity;
    } else {
      stream << " max_terminal_waypoint_abs_velocity=unavailable"
             << " max_terminal_waypoint_velocity_joint=unavailable"
             << " max_terminal_waypoint_velocity_signed=unavailable";
    }
    stream << " low_velocity_samples=" << low_velocity_samples_
           << " position_window_samples=" << position_window_samples_
           << " position_window_span_ms=" << position_window_span_ms()
           << " position_excursion=" << position_excursion_
           << " position_window_resets=" << position_window_resets_
           << " requirements=(target_error<=" << kTerminalPositionTolerance
           << " joint_units and (velocity<=" << kTerminalVelocityTolerance
           << " joint_units/s for " << kRequiredLowVelocitySamples
           << " samples or position_excursion<=" << kPositionStabilityTolerance
           << " joint_units for " << kPositionStabilityWindow.count()
           << " ms with at least " << kRequiredPositionStabilitySamples
           << " samples))";
    return stream.str();
  }

private:
  static const char * observation_label(const QuiescenceObservation observation) noexcept
  {
    switch (observation) {
      case QuiescenceObservation::kNone:
        return "none";
      case QuiescenceObservation::kStale:
        return "stale";
      case QuiescenceObservation::kNonMonotonic:
        return "non_monotonic";
      case QuiescenceObservation::kOutsideTarget:
        return "outside_target";
      case QuiescenceObservation::kAtTarget:
        return "at_target";
    }
    return "unknown";
  }

  void reset_evidence() noexcept
  {
    low_velocity_samples_ = 0U;
    position_window_start_.reset();
    position_window_samples_ = 0U;
    minimum_positions_.clear();
    maximum_positions_.clear();
    position_excursion_ = 0.0;
  }

  void begin_position_window(
    const std::vector<double> & positions,
    const std::chrono::steady_clock::time_point received_at)
  {
    position_window_start_ = received_at;
    latest_position_sample_ = received_at;
    position_window_samples_ = 1U;
    minimum_positions_ = positions;
    maximum_positions_ = positions;
    position_excursion_ = 0.0;
  }

  void observe_position_window(
    const std::vector<double> & positions,
    const std::chrono::steady_clock::time_point received_at)
  {
    if (!position_window_start_ ||
      minimum_positions_.size() != positions.size() ||
      received_at < latest_position_sample_)
    {
      begin_position_window(positions, received_at);
      return;
    }

    double candidate_excursion = 0.0;
    for (std::size_t index = 0U; index < positions.size(); ++index) {
      candidate_excursion = std::max(
        candidate_excursion,
        std::max(maximum_positions_[index], positions[index]) -
        std::min(minimum_positions_[index], positions[index]));
    }
    if (candidate_excursion > kPositionStabilityTolerance) {
      ++position_window_resets_;
      begin_position_window(positions, received_at);
      return;
    }

    for (std::size_t index = 0U; index < positions.size(); ++index) {
      minimum_positions_[index] = std::min(minimum_positions_[index], positions[index]);
      maximum_positions_[index] = std::max(maximum_positions_[index], positions[index]);
    }
    latest_position_sample_ = received_at;
    ++position_window_samples_;
    position_excursion_ = candidate_excursion;
  }

  [[nodiscard]] double position_window_span_ms() const noexcept
  {
    if (!position_window_start_ || position_window_samples_ < 2U) {
      return 0.0;
    }
    return std::chrono::duration<double, std::milli>(
      latest_position_sample_ - *position_window_start_).count();
  }

  QuiescenceObservation last_observation_{QuiescenceObservation::kNone};
  std::size_t examined_samples_{0U};
  std::size_t fresh_samples_{0U};
  std::size_t low_velocity_samples_{0U};
  std::size_t position_window_samples_{0U};
  std::size_t position_window_resets_{0U};
  std::int64_t previous_stamp_nanoseconds_{0};
  std::int64_t last_sample_stamp_nanoseconds_{0};
  std::int64_t last_ros_now_nanoseconds_{0};
  std::int64_t epoch_barrier_nanoseconds_{0};
  std::optional<TargetErrorEvidence> last_target_error_evidence_;
  std::optional<VelocityEvidence> last_velocity_evidence_;
  std::optional<std::chrono::steady_clock::time_point> position_window_start_;
  std::chrono::steady_clock::time_point latest_position_sample_{};
  std::vector<double> minimum_positions_;
  std::vector<double> maximum_positions_;
  double position_excursion_{0.0};
};

}  // namespace manipulation::moveit::detail

#endif  // QUIESCENCE_POLICY_HPP_
