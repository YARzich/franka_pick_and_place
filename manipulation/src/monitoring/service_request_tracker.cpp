// Responsibility: Implements tokenized lifecycle tracking for one asynchronous dependency request.

#include "service_request_tracker.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace manipulation::monitoring::detail
{

std::optional<ServiceRequestToken> ServiceRequestTracker::tryStart(
  const TimePoint now,
  const std::chrono::nanoseconds timeout)
{
  if (timeout <= Clock::duration::zero()) {
    throw std::invalid_argument("service request timeout must be positive");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return std::nullopt;
  }

  if (next_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("service request generation counter exhausted");
  }
  const ServiceRequestToken token{++next_generation_};
  const auto maximum_remaining = TimePoint::max() - now;
  const auto deadline = timeout > maximum_remaining ? TimePoint::max() : now + timeout;
  active_ = ActiveRequest{token, now, deadline, std::nullopt};
  return token;
}

bool ServiceRequestTracker::bindRequestId(
  const ServiceRequestToken token,
  const std::int64_t request_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || active_->token.generation != token.generation) {
    return false;
  }
  active_->request_id = request_id;
  return true;
}

ServiceResponseDisposition ServiceRequestTracker::acceptResponse(
  const ServiceRequestToken token,
  const TimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || active_->token.generation != token.generation) {
    return ServiceResponseDisposition::kStaleGeneration;
  }
  if (now >= active_->deadline) {
    active_.reset();
    ++timeout_count_;
    return ServiceResponseDisposition::kDeadlineExceeded;
  }
  active_.reset();
  return ServiceResponseDisposition::kAccept;
}

bool ServiceRequestTracker::abandon(const ServiceRequestToken token)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || active_->token.generation != token.generation) {
    return false;
  }
  active_.reset();
  return true;
}

std::optional<ExpiredServiceRequest> ServiceRequestTracker::expire(const TimePoint now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || now < active_->deadline) {
    return std::nullopt;
  }

  ExpiredServiceRequest expired{
    active_->token,
    active_->request_id,
    nonNegativeAge(now, active_->started_at)};
  active_.reset();
  ++timeout_count_;
  return expired;
}

ServiceRequestSnapshot ServiceRequestTracker::snapshot(const TimePoint now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ServiceRequestSnapshot result;
  result.timeout_count = timeout_count_;
  if (!active_) {
    return result;
  }
  result.in_flight = true;
  result.generation = active_->token.generation;
  result.request_id = active_->request_id;
  result.age = nonNegativeAge(now, active_->started_at);
  return result;
}

std::chrono::nanoseconds ServiceRequestTracker::nonNegativeAge(
  const TimePoint now,
  const TimePoint started_at)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::max(now - started_at, Clock::duration::zero()));
}

}  // namespace manipulation::monitoring::detail
