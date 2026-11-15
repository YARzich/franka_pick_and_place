// Responsibility: Converts validated sequence timeouts into saturating steady deadlines.

#ifndef WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_TIME_HPP_
#define WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_TIME_HPP_

#include <chrono>

namespace manipulation::workflow::detail
{

inline std::chrono::steady_clock::time_point sequence_saturating_deadline(
  const std::chrono::steady_clock::time_point start,
  const std::chrono::steady_clock::duration timeout)
{
  const auto maximum_timeout = std::chrono::steady_clock::time_point::max() - start;
  return timeout >= maximum_timeout ?
         std::chrono::steady_clock::time_point::max() : start + timeout;
}

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_SEQUENCE_ACTION_TIME_HPP_
