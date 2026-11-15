// Responsibility: Defines stale-response rejection and timeout accounting for monitored ROS
//   services.

#ifndef MONITORING__SERVICE_REQUEST_TRACKER_HPP_
#define MONITORING__SERVICE_REQUEST_TRACKER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>

namespace manipulation::monitoring::detail
{

/// Identifies one service request independently of the middleware request ID.
struct ServiceRequestToken
{
  std::uint64_t generation{0};
};

/// Result of validating a service response against its steady deadline.
enum class ServiceResponseDisposition
{
  kAccept,
  kDeadlineExceeded,
  kStaleGeneration,
};

/// Request removed from the active slot after its steady deadline elapsed.
struct ExpiredServiceRequest
{
  ServiceRequestToken token;
  std::optional<std::int64_t> request_id;
  std::chrono::nanoseconds age{};
};

/// Race-free operational state exposed through standard diagnostics.
struct ServiceRequestSnapshot
{
  bool in_flight{false};
  std::uint64_t generation{0};
  std::optional<std::int64_t> request_id;
  std::chrono::nanoseconds age{};
  std::uint64_t timeout_count{0};
};

/// Tracks one asynchronous service request using only a steady clock.
/**
 * A generation token separates middleware lifetime from application lifetime.
 * Once a request expires, its callback can no longer complete a newer request
 * or publish stale state even when the executor had already queued that callback.
 * The caller remains responsible for removing an expired middleware request.
 */
class ServiceRequestTracker final
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  /// Reserve the single request slot and calculate a checked steady deadline.
  [[nodiscard]] std::optional<ServiceRequestToken> tryStart(
    TimePoint now,
    std::chrono::nanoseconds timeout);

  /// Associate the token with the request ID returned by async_send_request().
  /**
   * Returns false when the token was already completed, abandoned or expired.
   * In that case the caller must attempt remove_pending_request(request_id).
   */
  [[nodiscard]] bool bindRequestId(
    ServiceRequestToken token,
    std::int64_t request_id);

  /// Accept only the current generation and reject responses past the deadline.
  [[nodiscard]] ServiceResponseDisposition acceptResponse(
    ServiceRequestToken token,
    TimePoint now);

  /// Release the slot after async_send_request() throws before returning an ID.
  [[nodiscard]] bool abandon(ServiceRequestToken token);

  /// Invalidate and return the active request when its deadline has elapsed.
  [[nodiscard]] std::optional<ExpiredServiceRequest> expire(TimePoint now);

  /// Return a steady-time snapshot without exposing mutable tracker state.
  [[nodiscard]] ServiceRequestSnapshot snapshot(TimePoint now) const;

private:
  struct ActiveRequest
  {
    ServiceRequestToken token;
    TimePoint started_at;
    TimePoint deadline;
    std::optional<std::int64_t> request_id;
  };

  [[nodiscard]] static std::chrono::nanoseconds nonNegativeAge(
    TimePoint now,
    TimePoint started_at);

  mutable std::mutex mutex_;
  std::optional<ActiveRequest> active_;
  std::uint64_t next_generation_{0};
  std::uint64_t timeout_count_{0};
};

}  // namespace manipulation::monitoring::detail

#endif  // MONITORING__SERVICE_REQUEST_TRACKER_HPP_
