// Responsibility: Implements fail-closed response classification and revision retry selection for
//   clients.

#include "manipulation/scene/scene_client_support.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace manipulation::scene
{

builtin_interfaces::msg::Duration duration_from_positive_seconds(
  const double seconds)
{
  constexpr auto nanoseconds_per_second = 1000000000LL;
  const auto first_unrepresentable_second =
    static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0;
  if (!std::isfinite(seconds) || seconds <= 0.0 ||
    seconds >= first_unrepresentable_second)
  {
    throw std::invalid_argument(
            "duration must be positive, finite and representable");
  }

  auto whole_seconds = static_cast<std::int64_t>(std::floor(seconds));
  auto nanoseconds = std::llround(
    (seconds - static_cast<double>(whole_seconds)) *
    static_cast<double>(nanoseconds_per_second));
  if (nanoseconds == nanoseconds_per_second) {
    ++whole_seconds;
    nanoseconds = 0;
  }
  if (whole_seconds > std::numeric_limits<std::int32_t>::max()) {
    throw std::invalid_argument("duration rounds beyond interface range");
  }

  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<std::int32_t>(whole_seconds);
  duration.nanosec = static_cast<std::uint32_t>(nanoseconds);
  return duration;
}

std::string make_scene_operation_id(
  const std::string & owner_id,
  const std::string & operation,
  const std::string & resource_id,
  const std::uint32_t entropy_high,
  const std::uint32_t entropy_low)
{
  if (owner_id.empty() || operation.empty() || resource_id.empty()) {
    throw std::invalid_argument(
            "operation ID components must not be empty");
  }

  std::ostringstream stream;
  stream << owner_id << '/' << operation << '/' << resource_id << '/'
         << std::hex << std::setfill('0')
         << std::setw(8) << entropy_high
         << std::setw(8) << entropy_low;
  return stream.str();
}

LeaseReleaseGuard::LeaseReleaseGuard(ReleaseCallback release)
: release_(std::move(release))
{
  if (!release_) {
    throw std::invalid_argument("release callback must not be empty");
  }
}

LeaseReleaseGuard::~LeaseReleaseGuard() noexcept
{
  if (active_) {
    (void)invoke_release();
  }
}

bool LeaseReleaseGuard::release() noexcept
{
  if (!active_) {
    return true;
  }
  if (!invoke_release()) {
    return false;
  }
  active_ = false;
  return true;
}

bool LeaseReleaseGuard::active() const noexcept
{
  return active_;
}

bool LeaseReleaseGuard::invoke_release() noexcept
{
  try {
    return release_();
  } catch (...) {
    return false;
  }
}

}  // namespace manipulation::scene
