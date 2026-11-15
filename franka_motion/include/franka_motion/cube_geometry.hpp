// Responsibility: Validates known cube dimensions without imposing a pose policy.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace franka_motion
{

/// Return whether all three finite positive dimensions describe equal cube edges.
inline bool has_cubic_dimensions(
  const std::array<double, 3> & dimensions,
  const double relative_tolerance = 1.0e-6) noexcept
{
  if (!std::isfinite(relative_tolerance) || relative_tolerance < 0.0 ||
    !std::all_of(
      dimensions.begin(), dimensions.end(),
      [](const double value) {return std::isfinite(value) && value > 0.0;}))
  {
    return false;
  }
  const double scale = *std::max_element(dimensions.begin(), dimensions.end());
  return std::abs(dimensions[0] - dimensions[1]) <= relative_tolerance * scale &&
         std::abs(dimensions[0] - dimensions[2]) <= relative_tolerance * scale;
}

}  // namespace franka_motion
