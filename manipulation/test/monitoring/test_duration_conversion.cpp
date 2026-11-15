// Responsibility: Verifies safe conversion and validation of ROS duration configuration values.

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "duration_conversion.hpp"
#include "gtest/gtest.h"

namespace manipulation::monitoring::detail
{
namespace
{

TEST(DurationConversion, ConvertsRepresentableSeconds)
{
  EXPECT_EQ(secondsToDuration(0.0, "timeout").count(), 0);
  EXPECT_EQ(secondsToDuration(1.25, "timeout").count(), 1'250'000'000);
}

TEST(DurationConversion, RejectsInvalidValues)
{
  EXPECT_THROW(secondsToDuration(-1.0, "timeout"), std::invalid_argument);
  EXPECT_THROW(
    secondsToDuration(std::numeric_limits<double>::infinity(), "timeout"),
    std::invalid_argument);
  EXPECT_THROW(
    secondsToDuration(std::numeric_limits<double>::quiet_NaN(), "timeout"),
    std::invalid_argument);
}

TEST(DurationConversion, RejectsValuesOutsideSignedNanosecondRange)
{
  constexpr long double kNanosecondsPerSecond = 1.0e9L;
  const long double max_seconds =
    static_cast<long double>(std::chrono::nanoseconds::max().count()) /
    kNanosecondsPerSecond;
  const double largest_safe_probe = std::nextafter(
    static_cast<double>(max_seconds), 0.0);
  const double first_unsafe_probe = std::nextafter(
    static_cast<double>(max_seconds), std::numeric_limits<double>::infinity());

  EXPECT_NO_THROW(secondsToDuration(largest_safe_probe, "timeout"));
  EXPECT_THROW(secondsToDuration(first_unsafe_probe, "timeout"), std::out_of_range);
  EXPECT_THROW(
    secondsToDuration(std::numeric_limits<double>::max(), "timeout"),
    std::out_of_range);
}

}  // namespace
}  // namespace manipulation::monitoring::detail
