// Responsibility: Defines client-side interpretation of coordinator revisions and mutation
//   responses.

#ifndef MANIPULATION__SCENE__SCENE_CLIENT_SUPPORT_HPP_
#define MANIPULATION__SCENE__SCENE_CLIENT_SUPPORT_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include <builtin_interfaces/msg/duration.hpp>

namespace manipulation::scene
{

/**
 * Convert positive finite seconds to a normalized ROS duration message.
 *
 * The message carries a relative interval, not a ROS-clock timestamp; clients
 * use it as a steady-clock lease TTL after the coordinator validates it.
 */
[[nodiscard]] builtin_interfaces::msg::Duration duration_from_positive_seconds(
  double seconds);

/**
 * Build a readable operation ID from caller-supplied entropy.
 *
 * Keeping entropy outside this function makes ID formatting deterministic and
 * lets each process choose an appropriate entropy source.
 */
[[nodiscard]] std::string make_scene_operation_id(
  const std::string & owner_id,
  const std::string & operation,
  const std::string & resource_id,
  std::uint32_t entropy_high,
  std::uint32_t entropy_low);

/**
 * Best-effort RAII guard for a coordinator lease.
 *
 * A failed explicit release leaves the guard active, so destruction performs
 * one final attempt. Callback exceptions are translated to failure and never
 * escape the destructor. The guard is intentionally not thread-safe; one
 * orchestration path owns both the callback and guard lifetime.
 */
class LeaseReleaseGuard final
{
public:
  using ReleaseCallback = std::function<bool()>;

  explicit LeaseReleaseGuard(ReleaseCallback release);
  ~LeaseReleaseGuard() noexcept;

  LeaseReleaseGuard(const LeaseReleaseGuard &) = delete;
  LeaseReleaseGuard & operator=(const LeaseReleaseGuard &) = delete;
  LeaseReleaseGuard(LeaseReleaseGuard &&) = delete;
  LeaseReleaseGuard & operator=(LeaseReleaseGuard &&) = delete;

  /** Release now. A failure keeps the guard active for a destructor retry. */
  [[nodiscard]] bool release() noexcept;

  /** Return whether this guard still owns release responsibility. */
  [[nodiscard]] bool active() const noexcept;

private:
  [[nodiscard]] bool invoke_release() noexcept;

  ReleaseCallback release_;
  bool active_{true};
};

}  // namespace manipulation::scene

#endif  // MANIPULATION__SCENE__SCENE_CLIENT_SUPPORT_HPP_
