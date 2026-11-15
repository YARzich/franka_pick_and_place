// Responsibility: Implements monotonic observation tracking and non-negative age classification.

#include "manipulation/monitoring/freshness_monitor.hpp"

#include <algorithm>
#include <stdexcept>

namespace manipulation::monitoring
{
namespace
{

std::chrono::nanoseconds nonNegativeAge(
  FreshnessMonitor::TimePoint newer,
  FreshnessMonitor::TimePoint older)
{
  return std::max(newer - older, FreshnessMonitor::Clock::duration::zero());
}

}  // namespace

FreshnessMonitor::FreshnessMonitor(FreshnessPolicy policy, TimePoint started_at)
: policy_(policy), started_at_(started_at)
{
  if (policy_.startup_grace < Clock::duration::zero() ||
    policy_.warn_after < Clock::duration::zero() ||
    policy_.stale_after < policy_.warn_after)
  {
    throw std::invalid_argument(
            "freshness thresholds require 0 <= warn_after <= stale_after and non-negative grace");
  }
}

void FreshnessMonitor::observe(TimePoint observed_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_observed_ = observed_at;
}

void FreshnessMonitor::reset(TimePoint started_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  started_at_ = started_at;
  last_observed_.reset();
}

FreshnessStatus FreshnessMonitor::status(TimePoint now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!last_observed_) {
    const auto startup_age = nonNegativeAge(now, started_at_);
    const bool within_grace = startup_age < policy_.startup_grace;
    return {
      within_grace ? HealthLevel::kWarn : HealthLevel::kStale,
      false,
      within_grace,
      startup_age};
  }

  const auto age = nonNegativeAge(now, *last_observed_);
  if (age >= policy_.stale_after) {
    return {HealthLevel::kStale, true, false, age};
  }
  if (age >= policy_.warn_after) {
    return {HealthLevel::kWarn, true, false, age};
  }
  return {HealthLevel::kOk, true, false, age};
}

}  // namespace manipulation::monitoring
