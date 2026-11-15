// Responsibility: Provides RAII ownership of a background ROS executor used by synchronous
//   scenarios.

#ifndef FRANKA_MOTION__EXECUTOR_SPINNER_HPP_
#define FRANKA_MOTION__EXECUTOR_SPINNER_HPP_

#include <atomic>
#include <exception>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace franka_motion
{

/**
 * RAII owner of the thread spinning a borrowed ROS executor.
 *
 * Nodes and the executor must outlive this object. Construction returns only
 * after spin has started or rethrows its startup failure. Destruction requests
 * executor cancellation and joins the thread. This helper is not thread-safe;
 * one orchestration thread owns its stop lifecycle.
 */
class ExecutorSpinner
{
public:
  explicit ExecutorSpinner(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {spin();})
  {
    waitUntilSpinning();
  }

  ExecutorSpinner(const ExecutorSpinner &) = delete;
  ExecutorSpinner & operator=(const ExecutorSpinner &) = delete;

  ~ExecutorSpinner()
  {
    stop();
  }

  /// Wake `spin()` without joining; ROS shutdown is an emergency fallback.
  void cancel() noexcept
  {
    try {
      executor_.cancel();
    } catch (...) {
      // Context shutdown is the only remaining way to wake spin() if its guard condition failed.
      try {
        if (rclcpp::ok()) {
          rclcpp::shutdown();
        }
      } catch (...) {
        // Keep noexcept destruction; a second failure here means the ROS context is already broken.
      }
    }
  }

  /// Join the owned thread after cancellation or external executor stop.
  void join() noexcept
  {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  /// Idempotently cancel the executor and join the owned thread.
  void stop() noexcept
  {
    cancel();
    join();
  }

private:
  void spin() noexcept
  {
    try {
      executor_.spin();
    } catch (...) {
      spin_exception_ = std::current_exception();
    }
    spin_finished_.store(true, std::memory_order_release);
  }

  void waitUntilSpinning()
  {
    while (!executor_.is_spinning()) {
      if (spin_finished_.load(std::memory_order_acquire)) {
        join();
        if (spin_exception_ != nullptr) {
          std::rethrow_exception(spin_exception_);
        }
        throw std::runtime_error("ROS executor stopped before entering spin");
      }
      std::this_thread::yield();
    }
  }

  rclcpp::Executor & executor_;
  std::atomic_bool spin_finished_{false};
  std::exception_ptr spin_exception_;
  std::thread thread_;
};

}  // namespace franka_motion

#endif  // FRANKA_MOTION__EXECUTOR_SPINNER_HPP_
