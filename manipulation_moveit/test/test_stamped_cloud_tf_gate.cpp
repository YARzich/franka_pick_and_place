// Responsibility: Verifies point clouds pass only with a transform valid at their exact stamp.

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "manipulation_moveit/stamped_cloud_tf_gate.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;

class ExecutorThread
{
public:
  explicit ExecutorThread(
    const std::vector<rclcpp::Node::SharedPtr> & nodes)
  {
    for (const auto & node : nodes) {
      executor_.add_node(node);
    }
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ExecutorThread(const ExecutorThread &) = delete;
  ExecutorThread & operator=(const ExecutorThread &) = delete;

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

bool waitUntil(
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

geometry_msgs::msg::TransformStamped identityTransform(
  const std::string & parent,
  const std::string & child,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.rotation.w = 1.0;
  return transform;
}

sensor_msgs::msg::PointCloud2 testCloud(
  const std::string & frame,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = frame;
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.point_step = 4;
  cloud.row_step = 4;
  cloud.data = {1U, 2U, 3U, 4U};
  cloud.is_dense = true;
  return cloud;
}

class CloudCapture
{
public:
  void store(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_cloud_ = *cloud;
      ++count_;
    }
    condition_.notify_all();
  }

  bool waitForCount(std::size_t expected, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, expected]() {
               return count_ >= expected;
    });
  }

  std::size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

  std::optional<sensor_msgs::msg::PointCloud2> lastCloud() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_cloud_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<sensor_msgs::msg::PointCloud2> last_cloud_;
  std::size_t count_{0};
};

class RclcppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

StampedCloudTfGateConfig validConfig()
{
  StampedCloudTfGateConfig config;
  config.input_topic = "/sensor/cloud";
  config.output_topic = "/moveit/cloud";
  config.expected_source_frame = "camera_optical_frame";
  config.required_frames = {"left_finger_link", "right_finger_link"};
  return config;
}

TEST(StampedCloudTfGateConfig, AcceptsBoundedExactStampContract)
{
  EXPECT_NO_THROW(validateStampedCloudTfGateConfig(validConfig()));
}

TEST(StampedCloudTfGateConfig, RejectsTopicLoop)
{
  auto config = validConfig();
  config.output_topic = config.input_topic;
  EXPECT_THROW(validateStampedCloudTfGateConfig(config), std::invalid_argument);
}

TEST(StampedCloudTfGateConfig, RejectsMissingOrDuplicateFrames)
{
  auto missing = validConfig();
  missing.required_frames.clear();
  EXPECT_THROW(validateStampedCloudTfGateConfig(missing), std::invalid_argument);

  auto duplicate = validConfig();
  duplicate.required_frames = {"tool_link", "tool_link"};
  EXPECT_THROW(validateStampedCloudTfGateConfig(duplicate), std::invalid_argument);
}

TEST(StampedCloudTfGateConfig, RejectsNonRelativeOrMalformedFrames)
{
  for (const auto & frame : {"/camera", "bad frame", "camera/", "camera//optical"}) {
    auto source = validConfig();
    source.expected_source_frame = frame;
    EXPECT_THROW(validateStampedCloudTfGateConfig(source), std::invalid_argument);

    auto target = validConfig();
    target.required_frames = {frame};
    EXPECT_THROW(validateStampedCloudTfGateConfig(target), std::invalid_argument);
  }
}

TEST(StampedCloudTfGateConfig, RejectsUnboundedQueue)
{
  auto config = validConfig();
  config.queue_size = 0;
  EXPECT_THROW(validateStampedCloudTfGateConfig(config), std::invalid_argument);
}

TEST(StampedCloudTfGateConfig, RejectsAmbiguousSourceAndTimeouts)
{
  auto config = validConfig();
  config.expected_source_frame.clear();
  EXPECT_THROW(validateStampedCloudTfGateConfig(config), std::invalid_argument);

  config = validConfig();
  config.transform_timeout_sec = 0.0;
  EXPECT_THROW(validateStampedCloudTfGateConfig(config), std::invalid_argument);

  config = validConfig();
  config.health_timeout_sec = 0.0;
  EXPECT_THROW(validateStampedCloudTfGateConfig(config), std::invalid_argument);
}

TEST_F(RclcppTest, ConstructsCompleteTfWaitInfrastructure)
{
  const auto options = rclcpp::NodeOptions()
    .append_parameter_override("input_topic", "/test/raw_cloud")
    .append_parameter_override("output_topic", "/test/gated_cloud")
    .append_parameter_override("expected_source_frame", "test_camera")
    .append_parameter_override(
      "required_frames", std::vector<std::string>{"left_finger_link"})
    .append_parameter_override("queue_size", 3);

  auto gate = std::make_shared<StampedCloudTfGate>(options);
  const auto update_result = gate->set_parameter(
    rclcpp::Parameter("expected_source_frame", "other_camera"));
  EXPECT_FALSE(update_result.successful);
}

TEST_F(RclcppTest, WaitsForEveryTfAndForwardsTheCloudUnchanged)
{
  constexpr char kInputTopic[] = "/test/stamped_cloud_gate/raw";
  constexpr char kOutputTopic[] = "/test/stamped_cloud_gate/gated";
  constexpr char kSourceFrame[] = "test_camera";
  const auto options = rclcpp::NodeOptions()
    .append_parameter_override("input_topic", kInputTopic)
    .append_parameter_override("output_topic", kOutputTopic)
    .append_parameter_override("expected_source_frame", kSourceFrame)
    .append_parameter_override(
      "required_frames", std::vector<std::string>{"test_left", "test_right"})
    .append_parameter_override("queue_size", 3)
    .append_parameter_override("transform_timeout_sec", 1.0)
    .append_parameter_override("health_timeout_sec", 2.0);

  auto gate = std::make_shared<StampedCloudTfGate>(options);
  auto io_node = std::make_shared<rclcpp::Node>("stamped_cloud_tf_gate_test_io");
  auto publisher = io_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInputTopic, rclcpp::SensorDataQoS());
  CloudCapture capture;
  auto subscription = io_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    kOutputTopic, rclcpp::SensorDataQoS(),
    [&capture](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
      capture.store(cloud);
    });
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(io_node);
  ExecutorThread executor({gate, io_node});

  ASSERT_TRUE(waitUntil(
      [&publisher, &subscription]() {
        return publisher->get_subscription_count() > 0U &&
               subscription->get_publisher_count() > 0U;
      }, 2s));

  const auto stamp = io_node->now();
  broadcaster->sendTransform(identityTransform("test_left", kSourceFrame, stamp));
  std::this_thread::sleep_for(100ms);

  const auto cloud = testCloud(kSourceFrame, stamp);
  publisher->publish(cloud);
  EXPECT_FALSE(capture.waitForCount(1U, 150ms));

  broadcaster->sendTransform(identityTransform("test_left", "test_right", stamp));
  ASSERT_TRUE(capture.waitForCount(1U, 1s));
  ASSERT_EQ(capture.count(), 1U);

  const auto forwarded = capture.lastCloud();
  ASSERT_TRUE(forwarded.has_value());
  EXPECT_EQ(forwarded->header.frame_id, cloud.header.frame_id);
  EXPECT_EQ(forwarded->header.stamp.sec, cloud.header.stamp.sec);
  EXPECT_EQ(forwarded->header.stamp.nanosec, cloud.header.stamp.nanosec);
  EXPECT_EQ(forwarded->height, cloud.height);
  EXPECT_EQ(forwarded->width, cloud.width);
  EXPECT_EQ(forwarded->point_step, cloud.point_step);
  EXPECT_EQ(forwarded->row_step, cloud.row_step);
  EXPECT_EQ(forwarded->data, cloud.data);
  EXPECT_EQ(forwarded->is_dense, cloud.is_dense);

  publisher->publish(testCloud(kSourceFrame, rclcpp::Time(0, 0, RCL_ROS_TIME)));
  publisher->publish(testCloud("unexpected_frame", io_node->now()));
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(capture.count(), 1U);
}

TEST_F(RclcppTest, RejectsMalformedStampsBeforeFilterAndRecoversWithValidCloud)
{
  constexpr char kInput[] = "/test/malformed_stamp/raw";
  constexpr char kOutput[] = "/test/malformed_stamp/gated";
  constexpr char kFrame[] = "stamp_test_frame";
  const auto options = rclcpp::NodeOptions()
    .append_parameter_override("input_topic", kInput)
    .append_parameter_override("output_topic", kOutput)
    .append_parameter_override("expected_source_frame", kFrame)
    .append_parameter_override("required_frames", std::vector<std::string>{kFrame});
  auto gate = std::make_shared<StampedCloudTfGate>(options);
  auto io = std::make_shared<rclcpp::Node>("malformed_stamp_test_io");
  auto publisher = io->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS());
  CloudCapture capture;
  auto subscription = io->create_subscription<sensor_msgs::msg::PointCloud2>(
    kOutput, rclcpp::SensorDataQoS(),
    [&capture](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {capture.store(cloud);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gate);
  executor.add_node(io);
  ASSERT_TRUE(waitUntil([&]() {
      executor.spin_some();
      return publisher->get_subscription_count() > 0U &&
             subscription->get_publisher_count() > 0U;
    }, 2s));

  auto cloud = testCloud(kFrame, io->now());
  const auto drain = [&]() {
      const auto deadline = std::chrono::steady_clock::now() + 150ms;
      while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(5ms);
      }
    };
  cloud.header.stamp.sec = -1;
  cloud.header.stamp.nanosec = 0U;
  publisher->publish(cloud);
  ASSERT_NO_THROW(drain());
  cloud.header.stamp.sec = 1;
  cloud.header.stamp.nanosec = 1000000000U;
  publisher->publish(cloud);
  ASSERT_NO_THROW(drain());
  EXPECT_EQ(capture.count(), 0U);

  cloud.header.stamp = io->now();
  publisher->publish(cloud);
  ASSERT_TRUE(waitUntil([&]() {
      executor.spin_some();
      return capture.count() == 1U;
    }, 1s));
  EXPECT_EQ(capture.lastCloud()->header.stamp, cloud.header.stamp);
}

}  // namespace
}  // namespace manipulation::moveit
