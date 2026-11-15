// Responsibility: Verifies fail-closed shape exclusion and atomic point-cloud octomap updates.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "fail_closed_pointcloud_octomap_updater.hpp"
#include "geometric_shapes/shapes.h"
#include "moveit/occupancy_map_monitor/occupancy_map_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.hpp"

namespace
{

using namespace std::chrono_literals;

occupancy_map_monitor::ShapeTransformCache transformCache(
  const std::initializer_list<occupancy_map_monitor::ShapeHandle> handles)
{
  occupancy_map_monitor::ShapeTransformCache cache;
  for (const auto handle : handles) {
    cache.emplace(handle, Eigen::Isometry3d::Identity());
  }
  return cache;
}

sensor_msgs::msg::PointCloud2 pointCloud(
  const rclcpp::Time & stamp,
  const float x)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "test_camera";
  cloud.header.stamp = stamp;
  cloud.height = 1U;
  cloud.width = 1U;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1U);
  sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
  *x_iterator = x;
  *y_iterator = 0.0F;
  *z_iterator = 0.0F;
  return cloud;
}

bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
  return predicate();
}

class FailClosedUpdaterRosTest : public ::testing::Test
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

TEST(FailClosedPointCloudOctomapUpdater, AcceptsOnlyExactNonEmptyShapeSet)
{
  const manipulation::moveit::ShapeHandleSet active_handles{1U, 2U, 3U};

  EXPECT_TRUE(manipulation::moveit::hasCompleteShapeTransformCache(
      active_handles, transformCache({1U, 2U, 3U})));
  EXPECT_FALSE(manipulation::moveit::hasCompleteShapeTransformCache(
      active_handles, transformCache({1U, 2U})));
  EXPECT_FALSE(manipulation::moveit::hasCompleteShapeTransformCache(
      active_handles, transformCache({1U, 2U, 3U, 4U})));
  EXPECT_FALSE(manipulation::moveit::hasCompleteShapeTransformCache(
      {}, transformCache({})));
}

TEST(FailClosedPointCloudOctomapUpdater, RejectsSameSizeWrongHandleSet)
{
  const manipulation::moveit::ShapeHandleSet active_handles{1U, 2U, 3U};

  EXPECT_FALSE(manipulation::moveit::hasCompleteShapeTransformCache(
      active_handles, transformCache({1U, 2U, 4U})));
}

TEST_F(FailClosedUpdaterRosTest, PluginDropsPartialCacheWithoutMutatingOctree)
{
  constexpr char kCloudTopic[] = "/test/fail_closed_octomap/cloud";
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter("sensors", std::vector<std::string>{"test_sensor"}),
      rclcpp::Parameter(
        "test_sensor.sensor_plugin",
        "manipulation_moveit/FailClosedPointCloudOctomapUpdater"),
      rclcpp::Parameter("test_sensor.point_cloud_topic", kCloudTopic),
      rclcpp::Parameter("test_sensor.max_range", 5.0),
      rclcpp::Parameter("test_sensor.padding_offset", 0.0),
      rclcpp::Parameter("test_sensor.padding_scale", 1.0),
      rclcpp::Parameter("test_sensor.point_subsample", 1),
      rclcpp::Parameter("test_sensor.max_update_rate", 0.0),
      rclcpp::Parameter("test_sensor.filtered_cloud_topic", "")});
  auto node = std::make_shared<rclcpp::Node>("fail_closed_octomap_plugin_test", options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  occupancy_map_monitor::OccupancyMapMonitor monitor(
    node, tf_buffer, "test_camera", 0.05);

  const auto first_handle = monitor.excludeShape(
    std::make_shared<shapes::Box>(0.1, 0.1, 0.1));
  const auto second_handle = monitor.excludeShape(
    std::make_shared<shapes::Box>(0.2, 0.2, 0.2));
  ASSERT_NE(first_handle, 0U);
  ASSERT_NE(second_handle, 0U);

  occupancy_map_monitor::ShapeTransformCache supplied_cache;
  std::size_t provider_calls = 0U;
  monitor.setTransformCacheCallback(
    [&supplied_cache, &provider_calls](
      const std::string &,
      const rclcpp::Time &,
      occupancy_map_monitor::ShapeTransformCache & cache)
    {
      ++provider_calls;
      cache = supplied_cache;
      return true;
    });
  std::size_t update_count = 0U;
  monitor.setUpdateCallback([&update_count]() {++update_count;});

  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    kCloudTopic, rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  monitor.startMonitor();
  ASSERT_TRUE(spinUntil(
      executor, [&publisher]() {return publisher->get_subscription_count() > 0U;}, 2s));

  publisher->publish(pointCloud(node->now(), 1.0F));
  ASSERT_TRUE(spinUntil(executor, [&provider_calls]() {return provider_calls >= 1U;}, 1s));
  EXPECT_EQ(update_count, 0U);
  EXPECT_EQ(monitor.getOcTreePtr()->size(), 0U);

  supplied_cache = transformCache({first_handle, second_handle});
  publisher->publish(pointCloud(node->now(), 1.0F));
  ASSERT_TRUE(spinUntil(executor, [&update_count]() {return update_count >= 1U;}, 1s));
  EXPECT_NE(monitor.getOcTreePtr()->search(1.0, 0.0, 0.0), nullptr);

  const auto accepted_tree_size = monitor.getOcTreePtr()->size();
  monitor.forgetShape(second_handle);
  publisher->publish(pointCloud(node->now(), 1.5F));
  ASSERT_TRUE(spinUntil(executor, [&provider_calls]() {return provider_calls >= 3U;}, 1s));
  EXPECT_EQ(update_count, 1U);
  EXPECT_EQ(monitor.getOcTreePtr()->size(), accepted_tree_size);

  supplied_cache = transformCache({first_handle});
  publisher->publish(pointCloud(node->now(), 1.5F));
  ASSERT_TRUE(spinUntil(executor, [&update_count]() {return update_count >= 2U;}, 1s));
  EXPECT_NE(monitor.getOcTreePtr()->search(1.5, 0.0, 0.0), nullptr);

  monitor.stopMonitor();
  executor.remove_node(node);
}

}  // namespace
