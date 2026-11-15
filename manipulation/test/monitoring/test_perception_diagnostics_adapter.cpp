// Responsibility: Verifies perception diagnostics conversion into stable typed health observations.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "manipulation/monitoring/health_aggregator.hpp"
#include "perception_diagnostics_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::monitoring::FreshnessMonitor;
using manipulation::monitoring::FreshnessPolicy;
using manipulation::monitoring::HealthComponentId;
using manipulation::monitoring::HealthLevel;
using manipulation::monitoring::PerceptionDiagnosticsAdapter;

constexpr char kTaskName[] = "perception/depth";
constexpr char kPrimaryStatus[] = "primary_depth: perception/depth";
constexpr char kAlternativeStatus[] = "fallback_depth: perception/depth";

class PerceptionDiagnosticsAdapterTest : public ::testing::Test
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

TEST_F(PerceptionDiagnosticsAdapterTest, RoutesAlternativeExactNamesToOneState)
{
  auto node = std::make_shared<rclcpp::Node>("perception_adapter_unit_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto started_at = FreshnessMonitor::Clock::now();
  PerceptionDiagnosticsAdapter adapter(
    *node,
    PerceptionDiagnosticsAdapter::Config{
      "/test/perception_diagnostics",
      {
        {kTaskName, HealthComponentId::kStereoPerception},
        {kTaskName, HealthComponentId::kStereoPerception}
      },
      {kPrimaryStatus, kAlternativeStatus},
      {kTaskName},
      FreshnessPolicy{0ms, 5s, 10s},
      started_at});
  auto publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/test/perception_diagnostics", 10);

  for (int attempt = 0; attempt < 100 && publisher->get_subscription_count() == 0U;
    ++attempt)
  {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1U);

  const auto publish = [&](const std::string & name, std::uint8_t level) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = name;
      status.level = level;
      status.message = "unit-test status";
      diagnostic_msgs::msg::DiagnosticArray array;
      array.status.push_back(status);
      publisher->publish(array);
    };
  const auto spin_cycles = [&executor](int count) {
      for (int cycle = 0; cycle < count; ++cycle) {
        executor.spin_some();
        std::this_thread::sleep_for(5ms);
      }
    };

  publish(std::string("foreign: ") + kAlternativeStatus, 0U);
  publish(std::string(kAlternativeStatus) + "/suffix", 0U);
  spin_cycles(10);
  auto snapshot = adapter.snapshot(FreshnessMonitor::Clock::now());
  ASSERT_EQ(snapshot.size(), 1U);
  EXPECT_FALSE(snapshot.front().freshness.observed);

  publish(kAlternativeStatus, 0U);
  spin_cycles(10);
  snapshot = adapter.snapshot(FreshnessMonitor::Clock::now());
  ASSERT_EQ(snapshot.size(), 1U);
  EXPECT_TRUE(snapshot.front().freshness.observed);
  EXPECT_EQ(snapshot.front().reported_level, HealthLevel::kOk);

  publish(kPrimaryStatus, 1U);
  spin_cycles(10);
  snapshot = adapter.snapshot(FreshnessMonitor::Clock::now());
  ASSERT_EQ(snapshot.size(), 1U);
  EXPECT_TRUE(snapshot.front().required);
  EXPECT_EQ(snapshot.front().reported_level, HealthLevel::kWarn);

  executor.remove_node(node);
}

}  // namespace
