// Responsibility: Verifies startup grace and age-based health classification for observations.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "manipulation/monitoring/freshness_monitor.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation::monitoring::FreshnessMonitor;
using manipulation::monitoring::FreshnessPolicy;
using manipulation::monitoring::HealthLevel;

TEST(FreshnessMonitor, ReportsStartupGraceBeforeFirstObservation)
{
  const FreshnessMonitor::TimePoint start{};
  FreshnessMonitor monitor({2s, 500ms, 1s}, start);

  const auto starting = monitor.status(start + 1500ms);
  EXPECT_EQ(starting.level, HealthLevel::kWarn);
  EXPECT_FALSE(starting.observed);
  EXPECT_TRUE(starting.startup_grace_active);

  const auto missing = monitor.status(start + 2s);
  EXPECT_EQ(missing.level, HealthLevel::kStale);
  EXPECT_FALSE(missing.startup_grace_active);
}

TEST(FreshnessMonitor, ClassifiesObservationAgeAtExactBoundaries)
{
  const FreshnessMonitor::TimePoint start{};
  FreshnessMonitor monitor({0s, 500ms, 1s}, start);
  monitor.observe(start + 100ms);

  EXPECT_EQ(monitor.status(start + 599ms).level, HealthLevel::kOk);
  EXPECT_EQ(monitor.status(start + 600ms).level, HealthLevel::kWarn);
  EXPECT_EQ(monitor.status(start + 1100ms).level, HealthLevel::kStale);
}

TEST(FreshnessMonitor, ResetStartsASeparateGraceInterval)
{
  const FreshnessMonitor::TimePoint start{};
  FreshnessMonitor monitor({1s, 100ms, 200ms}, start);
  monitor.observe(start + 10ms);
  monitor.reset(start + 5s);

  const auto status = monitor.status(start + 5500ms);
  EXPECT_FALSE(status.observed);
  EXPECT_TRUE(status.startup_grace_active);
}

TEST(FreshnessMonitor, RejectsInvalidThresholds)
{
  EXPECT_THROW(FreshnessMonitor(FreshnessPolicy{0s, 2s, 1s}), std::invalid_argument);
  EXPECT_THROW(FreshnessMonitor(FreshnessPolicy{-1s, 1s, 2s}), std::invalid_argument);
}

TEST(FreshnessMonitor, ClampsInjectedBackwardTimeToZeroAge)
{
  const FreshnessMonitor::TimePoint start{10s};
  FreshnessMonitor monitor({0s, 1s, 2s}, start);
  monitor.observe(start);

  const auto status = monitor.status(FreshnessMonitor::TimePoint{5s});
  EXPECT_EQ(status.level, HealthLevel::kOk);
  EXPECT_EQ(status.age, 0ns);
}

}  // namespace
