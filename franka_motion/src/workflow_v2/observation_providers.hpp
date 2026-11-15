// Responsibility: Declares configured and detected V2 observation adapters.

#ifndef WORKFLOW_V2__OBSERVATION_PROVIDERS_HPP_
#define WORKFLOW_V2__OBSERVATION_PROVIDERS_HPP_

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <cstdint>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/buffer.hpp"

#include "workflow_v2/detected_pose_buffer.hpp"
#include "manipulation/workflow/transfer_workflow_ports.hpp"

namespace franka_motion::workflow_v2
{

/** One fixed world pose owned by a configured-observation preset, not the catalog. */
struct ConfiguredObservation
{
  std::string frame_id;
  geometry_msgs::msg::Pose pose;
  std::optional<double> confidence;
};

using ConfiguredObservationPreset = std::map<std::string, ConfiguredObservation>;

/** Immediate provider that stamps a configured world pose at observation time. */
class ConfiguredObservationProvider final : public manipulation::workflow::ObservationProvider
{
public:
  using StampSource = std::function<builtin_interfaces::msg::Time()>;

  ConfiguredObservationProvider(
    ConfiguredObservationPreset observations,
    StampSource stamp_source);

  [[nodiscard]] manipulation::workflow::ObservationProviderResult observe(
    const manipulation::workflow::ObjectInstance & instance,
    const manipulation::workflow::ObjectProfile & profile,
    manipulation::workflow::TransferExecutionContext & context) override;

  void request_stop() noexcept override;

private:
  ConfiguredObservationPreset observations_;
  StampSource stamp_source_;
  std::atomic<std::uint64_t> stop_generation_{0U};
};

/** Deployment-only routing for the current single-topic perception buffer. */
struct DetectedObservationProviderConfig
{
  std::string object_id;
  std::string target_frame;
  /// Optional lower bound for the observed position z in target_frame.
  /// Millimeter-level detector noise must not sink a supported object below
  /// its resting surface: a semantic pose inside the tabletop collision box
  /// turns the legal resting contact into a start-state collision. The
  /// configured-observation preset keeps the same clearance by construction.
  std::optional<double> minimum_position_z;
  /// Maximum measurement age in ROS seconds, including inference and TF latency.
  double max_observation_age_sec{5.0};
};

/**
 * Adapt the current stamped pose buffer and exact-stamp TF to ObservationProvider.
 *
 * Observation and TF waits have no semantic deadline. A private short wait slice
 * only makes pause, cancel, shutdown and request_stop observable.
 * One lifecycle owner serializes observe calls. Stamps must advance within a ROS
 * clock epoch; receipt freshness alone never establishes measurement freshness.
 */
class DetectedObservationProvider final : public manipulation::workflow::ObservationProvider
{
public:
  using StampSource = ConfiguredObservationProvider::StampSource;
  DetectedObservationProvider(
    DetectedPoseBuffer & pose_buffer,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    DetectedObservationProviderConfig config,
    StampSource stamp_source);

  [[nodiscard]] manipulation::workflow::ObservationProviderResult observe(
    const manipulation::workflow::ObjectInstance & instance,
    const manipulation::workflow::ObjectProfile & profile,
    manipulation::workflow::TransferExecutionContext & context) override;

  void request_stop() noexcept override;

private:
  [[nodiscard]] bool stopped(
    const manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t request_generation) const noexcept;

  [[nodiscard]] bool fresh_stamp(
    const builtin_interfaces::msg::Time & stamp, std::int64_t now) const noexcept;
  [[nodiscard]] std::int64_t observe_clock(bool & reset);

  DetectedPoseBuffer & pose_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  DetectedObservationProviderConfig config_;
  StampSource stamp_source_;
  std::optional<std::int64_t> last_ros_now_;
  std::optional<std::int64_t> consumed_stamp_;
  std::atomic<std::uint64_t> stop_generation_{0U};
};

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__OBSERVATION_PROVIDERS_HPP_
