// Responsibility: Converts validated floating-point seconds into bounded steady-clock durations.

#include "duration_conversion.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace manipulation::monitoring::detail
{

std::chrono::nanoseconds secondsToDuration(
  double value, const std::string & parameter_name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(parameter_name + " must be a finite non-negative value");
  }

  constexpr long double kNanosecondsPerSecond = 1.0e9L;
  const auto max_nanoseconds = std::chrono::nanoseconds::max().count();
  const long double max_seconds =
    static_cast<long double>(max_nanoseconds) / kNanosecondsPerSecond;
  if (static_cast<long double>(value) > max_seconds) {
    throw std::out_of_range(parameter_name + " exceeds the signed nanosecond range");
  }

  // The range check makes duration_cast's conversion to int64 well-defined.
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<long double>(value));
}

}  // namespace manipulation::monitoring::detail
