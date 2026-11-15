// Responsibility: Forwards one bounded request stop signal to a blocking subsystem.

#ifndef MTC__MTC_STOP_MONITOR_HPP_
#define MTC__MTC_STOP_MONITOR_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <utility>

namespace franka_motion::detail
{

using MtcStopRequested = std::function<bool()>;
using MtcDeadline = std::chrono::steady_clock::time_point;

/** Combine an outer cancellation callback with one steady-time deadline. */
inline MtcStopRequested make_bounded_stop_request(
  MtcStopRequested stop_requested, const MtcDeadline deadline)
{
  return [stop_requested = std::move(stop_requested), deadline]() {
           return std::chrono::steady_clock::now() >= deadline ||
                  (stop_requested && stop_requested());
         };
}

/** Restrict a local blocking wait without extending its enclosing request. */
inline MtcDeadline bounded_deadline(
  const MtcDeadline outer_deadline,
  const std::chrono::steady_clock::duration local_timeout)
{
  const auto now = std::chrono::steady_clock::now();
  if (local_timeout <= std::chrono::steady_clock::duration::zero()) {
    return std::min(outer_deadline, now);
  }
  const auto safe_timeout = std::min(local_timeout, MtcDeadline::max() - now);
  return std::min(outer_deadline, now + safe_timeout);
}

/**
 * Poll cancellation while a blocking API owns the caller thread.
 *
 * Both callbacks must remain valid until `finish()` or destruction joins the
 * polling thread. The callback is cooperative: this monitor cannot force a
 * third-party API to return when its own stop operation is ignored.
 */
class MtcStopMonitor final
{
public:
  using StopRequested = MtcStopRequested;
  using OnStop = std::function<void()>;

  MtcStopMonitor(StopRequested stop_requested, OnStop on_stop)
  : stop_requested_(std::move(stop_requested)), on_stop_(std::move(on_stop))
  {
    if (stop_requested_ && on_stop_) {
      worker_ = std::thread([this]() {run();});
    }
  }

  ~MtcStopMonitor()
  {
    finish();
  }

  MtcStopMonitor(const MtcStopMonitor &) = delete;
  MtcStopMonitor & operator=(const MtcStopMonitor &) = delete;

  void finish() noexcept
  {
    finished_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run() noexcept
  {
    while (!finished_.load(std::memory_order_acquire)) {
      bool stop = false;
      try {
        stop = stop_requested_();
      } catch (...) {
        stop = true;
      }
      if (stop) {
        try {
          on_stop_();
        } catch (...) {
          // Cancellation must never terminate the process from this thread.
        }
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
  }

  StopRequested stop_requested_;
  OnStop on_stop_;
  std::atomic<bool> finished_{false};
  std::thread worker_;
};

}  // namespace franka_motion::detail

#endif  // MTC__MTC_STOP_MONITOR_HPP_
