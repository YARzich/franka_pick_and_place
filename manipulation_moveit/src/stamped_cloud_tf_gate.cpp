// Responsibility: Validates cloud timestamps and forwards only samples with available
//   planning-frame TF.

#include "manipulation_moveit/stamped_cloud_tf_gate.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "manipulation/validation/frame_id.hpp"
#include "manipulation/validation/rigid_pose.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2_ros/create_timer_ros.hpp"

namespace manipulation::moveit
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor readOnlyParameter(
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

std::int64_t steadyNowNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double elapsedSeconds(std::int64_t since_ns, std::int64_t now_ns)
{
  if (since_ns <= 0 || now_ns < since_ns) {
    return 0.0;
  }
  return static_cast<double>(now_ns - since_ns) / 1.0e9;
}

}  // namespace

void validateStampedCloudTfGateConfig(const StampedCloudTfGateConfig & config)
{
  if (config.input_topic.empty()) {
    throw std::invalid_argument("stamped cloud TF gate input_topic must not be empty");
  }
  if (config.output_topic.empty()) {
    throw std::invalid_argument("stamped cloud TF gate output_topic must not be empty");
  }
  if (config.input_topic == config.output_topic) {
    throw std::invalid_argument("stamped cloud TF gate topics must be different");
  }
  if (!manipulation::validation::is_canonical_relative_frame_id(
      config.expected_source_frame))
  {
    throw std::invalid_argument(
            "stamped cloud TF gate expected_source_frame must be a canonical relative frame ID");
  }
  if (config.required_frames.empty()) {
    throw std::invalid_argument("stamped cloud TF gate requires at least one target frame");
  }
  if (config.queue_size == 0U) {
    throw std::invalid_argument("stamped cloud TF gate queue_size must be positive");
  }
  if (!std::isfinite(config.transform_timeout_sec) ||
    config.transform_timeout_sec <= 0.0)
  {
    throw std::invalid_argument(
            "stamped cloud TF gate transform_timeout_sec must be finite and positive");
  }
  if (!std::isfinite(config.health_timeout_sec) || config.health_timeout_sec <= 0.0) {
    throw std::invalid_argument(
            "stamped cloud TF gate health_timeout_sec must be finite and positive");
  }

  std::set<std::string> unique_frames;
  for (const auto & frame : config.required_frames) {
    if (!manipulation::validation::is_canonical_relative_frame_id(frame)) {
      throw std::invalid_argument(
              "stamped cloud TF gate target frames must be canonical relative frame IDs");
    }
    if (!unique_frames.insert(frame).second) {
      throw std::invalid_argument("stamped cloud TF gate frame names must be unique");
    }
  }
}

StampedCloudTfGate::StampedCloudTfGate(const rclcpp::NodeOptions & options)
: Node("moveit_octomap_tf_gate", options), config_(readConfig())
{
  validateStampedCloudTfGateConfig(config_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  // A dedicated TF executor lets MessageFilter wait while the node callback
  // publishes a large cloud without starving /tf and /tf_static delivery.
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_, this, true);

  cloud_publisher_ = create_publisher<PointCloud>(
    config_.output_topic, rclcpp::SensorDataQoS());
  cloud_subscriber_ = std::make_unique<message_filters::Subscriber<PointCloud>>(
    this, config_.input_topic, rmw_qos_profile_sensor_data);
  cloud_filter_ = std::make_unique<tf2_ros::MessageFilter<PointCloud>>(
    *tf_buffer_, config_.required_frames.front(), config_.queue_size,
    get_node_logging_interface(), get_node_clock_interface(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.transform_timeout_sec)));
  cloud_filter_->setTargetFrames(config_.required_frames);
  cloud_filter_->registerCallback(
    std::bind(&StampedCloudTfGate::forwardCloud, this, std::placeholders::_1));
  cloud_subscriber_->registerCallback(
    std::bind(&StampedCloudTfGate::receiveCloud, this, std::placeholders::_1));

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("moveit_octomap_input");
  diagnostics_->add("Octomap exact-stamp TF gate", this, &StampedCloudTfGate::diagnose);

  RCLCPP_INFO(
    get_logger(),
    "Octomap TF gate: %s -> %s, source_frame=%s, queue=%u, tf_timeout=%.3f s, "
    "required_frames=%s",
    config_.input_topic.c_str(), config_.output_topic.c_str(),
    config_.expected_source_frame.c_str(), config_.queue_size,
    config_.transform_timeout_sec, cloud_filter_->getTargetFramesString().c_str());
}

StampedCloudTfGateConfig StampedCloudTfGate::readConfig()
{
  StampedCloudTfGateConfig config;
  config.input_topic = declare_parameter<std::string>(
    "input_topic", "~/input",
    readOnlyParameter("Raw workspace-filtered sensor cloud"));
  config.output_topic = declare_parameter<std::string>(
    "output_topic", "~/output",
    readOnlyParameter("Cloud admitted to MoveIt's Octomap updater"));
  config.expected_source_frame = declare_parameter<std::string>(
    "expected_source_frame", "",
    readOnlyParameter("Required physical sensor frame for Octomap ray casting"));
  config.required_frames = declare_parameter<std::vector<std::string>>(
    "required_frames", {},
    readOnlyParameter("Robot leaf frames that must have exact-stamp TF"));
  const auto queue_size = declare_parameter<int>(
    "queue_size", 30, readOnlyParameter("Maximum number of pending sensor clouds"));
  if (queue_size <= 0) {
    throw std::invalid_argument("stamped cloud TF gate queue_size must be positive");
  }
  config.queue_size = static_cast<std::uint32_t>(queue_size);
  config.transform_timeout_sec = declare_parameter<double>(
    "transform_timeout_sec", 0.5,
    readOnlyParameter("Maximum exact-stamp TF wait for one cloud"));
  config.health_timeout_sec = declare_parameter<double>(
    "health_timeout_sec", 1.0,
    readOnlyParameter("Maximum healthy age of input and forwarded clouds"));
  return config;
}

void StampedCloudTfGate::receiveCloud(const PointCloud::ConstSharedPtr & cloud)
{
  received_count_.fetch_add(1, std::memory_order_relaxed);
  last_input_steady_ns_.store(steadyNowNanoseconds(), std::memory_order_relaxed);
  if (cloud->header.stamp.sec == 0 && cloud->header.stamp.nanosec == 0U) {
    zero_stamp_rejection_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Octomap cloud rejected: zero stamp would request latest TF");
    return;
  }
  if (!manipulation::validation::is_normalized_nonzero_stamp(cloud->header.stamp)) {
    invalid_stamp_rejection_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Octomap cloud rejected: malformed stamp sec=%d nanosec=%u",
      cloud->header.stamp.sec, cloud->header.stamp.nanosec);
    return;
  }
  if (cloud->header.frame_id != config_.expected_source_frame) {
    frame_rejection_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Octomap cloud rejected: source frame '%s' does not match expected '%s'",
      cloud->header.frame_id.c_str(), config_.expected_source_frame.c_str());
    return;
  }
  cloud_filter_->add(cloud);
}

void StampedCloudTfGate::forwardCloud(const PointCloud::ConstSharedPtr & cloud)
{
  forwarded_count_.fetch_add(1, std::memory_order_relaxed);
  last_forwarded_input_count_.store(
    received_count_.load(std::memory_order_relaxed), std::memory_order_relaxed);
  last_forward_steady_ns_.store(steadyNowNanoseconds(), std::memory_order_relaxed);
  cloud_publisher_->publish(*cloud);
}

void StampedCloudTfGate::diagnose(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto received = received_count_.load(std::memory_order_relaxed);
  const auto forwarded = forwarded_count_.load(std::memory_order_relaxed);
  const auto zero_stamp_rejections =
    zero_stamp_rejection_count_.load(std::memory_order_relaxed);
  const auto frame_rejections = frame_rejection_count_.load(std::memory_order_relaxed);
  const auto last_forwarded_input =
    last_forwarded_input_count_.load(std::memory_order_relaxed);
  const auto outstanding = received >= last_forwarded_input ?
    received - last_forwarded_input : 0U;
  const auto now_ns = steadyNowNanoseconds();
  const auto input_age_sec = elapsedSeconds(
    last_input_steady_ns_.load(std::memory_order_relaxed), now_ns);
  const auto forward_age_sec = elapsedSeconds(
    last_forward_steady_ns_.load(std::memory_order_relaxed), now_ns);
  const bool forwarding_blocked =
    forwarded == 0U || forward_age_sec > config_.health_timeout_sec ||
    outstanding >= config_.queue_size;

  status.add("input_topic", config_.input_topic);
  status.add("output_topic", config_.output_topic);
  status.add("expected_source_frame", config_.expected_source_frame);
  status.add("queue_size", config_.queue_size);
  status.add("transform_timeout_sec", config_.transform_timeout_sec);
  status.add("health_timeout_sec", config_.health_timeout_sec);
  status.add("received_clouds", received);
  status.add("forwarded_clouds", forwarded);
  status.add("zero_stamp_rejections", zero_stamp_rejections);
  status.add("invalid_stamp_rejections",
    invalid_stamp_rejection_count_.load(std::memory_order_relaxed));
  status.add("source_frame_rejections", frame_rejections);
  status.add("pending_clouds", outstanding);
  status.add("last_input_age_sec", input_age_sec);
  status.add("last_forward_age_sec", forward_age_sec);

  if (received == 0U) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "waiting for the first Octomap cloud");
  } else if (input_age_sec > config_.health_timeout_sec) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Octomap cloud input is stale");
  } else if (forwarding_blocked) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "cloud forwarding is blocked by incomplete exact-stamp robot TF");
  } else {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "exact-stamp robot TF contract is satisfied");
  }
}

}  // namespace manipulation::moveit
