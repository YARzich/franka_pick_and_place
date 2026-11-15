// Responsibility: Gates PointCloud2 delivery until its stamped sensor frame is transformable by
//   MoveIt.

#ifndef MANIPULATION_MOVEIT__STAMPED_CLOUD_TF_GATE_HPP_
#define MANIPULATION_MOVEIT__STAMPED_CLOUD_TF_GATE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "message_filters/subscriber.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace manipulation::moveit
{

/// Immutable ROS contract for the exact-stamp point-cloud admission gate.
struct StampedCloudTfGateConfig
{
  std::string input_topic;
  std::string output_topic;
  std::string expected_source_frame;
  std::vector<std::string> required_frames;
  std::uint32_t queue_size{30};
  double transform_timeout_sec{0.5};
  double health_timeout_sec{1.0};
};

/// Reject invalid or ambiguous topic/frame configurations at startup.
void validateStampedCloudTfGateConfig(const StampedCloudTfGateConfig & config);

/**
 * Fail-closed adapter between a sensor cloud and MoveIt's Octomap updater.
 *
 * tf2_ros::MessageFilter forwards a cloud only when its source frame can be
 * transformed to every configured robot frame at the cloud's exact stamp.
 * Together with MoveIt's bounded shape-TF lookup wait, this protects against a
 * MoveIt 2.12 self-filter race where a partial transform cache can otherwise
 * admit robot points. Messages are forwarded unchanged, preserving the
 * physical sensor origin used for ray casting.
 *
 * The node owns the TF buffer/listener, filter, publisher and diagnostics. TF
 * callbacks may run on the listener's dedicated executor thread; counters are
 * atomic and no cloud is retained after its callback. Health ages use steady
 * time, while TF matching uses the cloud's ROS stamp; queue size and the filter
 * timeout bound retained messages. Invalid headers or missing exact-stamp
 * transforms drop that cloud without mutating the downstream Octomap.
 */
class StampedCloudTfGate final : public rclcpp::Node
{
public:
  explicit StampedCloudTfGate(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud = sensor_msgs::msg::PointCloud2;

  [[nodiscard]] StampedCloudTfGateConfig readConfig();
  void receiveCloud(const PointCloud::ConstSharedPtr & cloud);
  void forwardCloud(const PointCloud::ConstSharedPtr & cloud);
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

  StampedCloudTfGateConfig config_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<message_filters::Subscriber<PointCloud>> cloud_subscriber_;
  std::unique_ptr<tf2_ros::MessageFilter<PointCloud>> cloud_filter_;
  rclcpp::Publisher<PointCloud>::SharedPtr cloud_publisher_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;

  std::atomic<std::uint64_t> received_count_{0};
  std::atomic<std::uint64_t> forwarded_count_{0};
  std::atomic<std::uint64_t> zero_stamp_rejection_count_{0};
  std::atomic<std::uint64_t> invalid_stamp_rejection_count_{0};
  std::atomic<std::uint64_t> frame_rejection_count_{0};
  std::atomic<std::uint64_t> last_forwarded_input_count_{0};
  std::atomic<std::int64_t> last_input_steady_ns_{0};
  std::atomic<std::int64_t> last_forward_steady_ns_{0};
};

}  // namespace manipulation::moveit

#endif  // MANIPULATION_MOVEIT__STAMPED_CLOUD_TF_GATE_HPP_
