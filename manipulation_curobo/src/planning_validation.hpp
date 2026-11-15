// Responsibility: Provides side-effect-free numeric validation shared by cuRobo request adapters.

#ifndef PLANNING_VALIDATION_HPP_
#define PLANNING_VALIDATION_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace manipulation_curobo::curobo::io::detail
{

inline bool allFinite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
             return std::isfinite(value);
  });
}

inline bool validScale(const double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

}  // namespace manipulation_curobo::curobo::io::detail

#endif  // PLANNING_VALIDATION_HPP_
