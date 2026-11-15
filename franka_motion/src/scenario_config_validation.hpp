// Responsibility: Provides strict scalar, array and duration validation for scenario configuration.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace franka_motion::config_validation
{

[[noreturn]] inline void invalidParameter(
  const std::string & name,
  const std::string & requirement)
{
  throw std::invalid_argument("Parameter '" + name + "' " + requirement);
}

inline void requireNonEmpty(const std::string & value, const std::string & name)
{
  if (value.empty()) {
    invalidParameter(name, "must not be empty");
  }
}

inline void requireFinite(double value, const std::string & name)
{
  if (!std::isfinite(value)) {
    invalidParameter(name, "must be finite");
  }
}

inline void requirePositive(double value, const std::string & name)
{
  requireFinite(value, name);
  if (value <= 0.0) {
    invalidParameter(name, "must be greater than zero");
  }
}

inline void requireNonNegative(double value, const std::string & name)
{
  requireFinite(value, name);
  if (value < 0.0) {
    invalidParameter(name, "must be non-negative");
  }
}

inline void requireUnitInterval(double value, const std::string & name)
{
  requireFinite(value, name);
  if (value < 0.0 || value > 1.0) {
    invalidParameter(name, "must be in [0, 1]");
  }
}

inline std::chrono::steady_clock::duration checkedDuration(
  double seconds,
  const std::string & name,
  bool allow_zero)
{
  if (allow_zero) {
    requireNonNegative(seconds, name);
  } else {
    requirePositive(seconds, name);
  }

  using Duration = std::chrono::steady_clock::duration;
  const double max_seconds = std::chrono::duration<double>(Duration::max()).count();
  if (seconds >= max_seconds) {
    invalidParameter(name, "exceeds the steady-clock duration range");
  }

  auto duration = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
  if (seconds > 0.0 && duration == Duration::zero()) {
    duration = Duration{1};
  }
  return duration;
}

template<std::size_t Size>
std::array<double, Size> checkedFiniteArray(
  const std::vector<double> & values,
  const std::string & name)
{
  if (values.size() != Size) {
    invalidParameter(name, "must contain exactly " + std::to_string(Size) + " values");
  }

  std::array<double, Size> result{};
  std::copy(values.begin(), values.end(), result.begin());
  if (!std::all_of(result.begin(), result.end(), [](double value) {
      return std::isfinite(value);
      }))
  {
    invalidParameter(name, "must contain only finite values");
  }
  return result;
}

}  // namespace franka_motion::config_validation
