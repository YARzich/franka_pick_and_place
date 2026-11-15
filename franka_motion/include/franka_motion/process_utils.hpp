// Responsibility: Provides RAII supervision of one-shot helper processes used by demo
//   orchestration.

#pragma once

#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

extern char ** environ;

namespace franka_motion
{
namespace detail
{
template<typename SetFlags, typename SetProcessGroup>
int configureIsolatedSpawnAttributes(
  posix_spawnattr_t * attributes,
  SetFlags && set_flags,
  SetProcessGroup && set_process_group)
{
  const int flags_result = set_flags(attributes, POSIX_SPAWN_SETPGROUP);
  if (flags_result != 0) {
    return flags_result;
  }
  return set_process_group(attributes, 0);
}

inline int configureIsolatedSpawnAttributes(posix_spawnattr_t * attributes)
{
  return configureIsolatedSpawnAttributes(
    attributes,
    posix_spawnattr_setflags,
    posix_spawnattr_setpgroup);
}

template<typename WaitPid>
pid_t waitPidRetryingInterrupt(
  const pid_t pid,
  int * status,
  const int options,
  WaitPid && wait_pid)
{
  pid_t result = -1;
  do {
    result = wait_pid(pid, status, options);
  } while (result < 0 && errno == EINTR);
  return result;
}

inline pid_t waitPidRetryingInterrupt(
  const pid_t pid,
  int * status,
  const int options)
{
  return waitPidRetryingInterrupt(pid, status, options, waitpid);
}
}  // namespace detail

/// Join argv for operator logging only; the result must not be executed by a shell.
inline std::string joinCommand(const std::vector<std::string> & command)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < command.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << command[i];
  }
  return stream.str();
}

/// Grace windows used before the mandatory SIGKILL reap.
struct ScopedProcessStopPolicy
{
  std::chrono::milliseconds interrupt_grace{2500};
  std::chrono::milliseconds terminate_grace{1500};
  std::chrono::milliseconds poll_interval{100};
};

/**
 * RAII owner of one short-lived child process group.
 *
 * Commands are spawned directly from argv, never through a shell. The class
 * owns and reaps the child PID and signals its process group during stop or
 * destruction. It is intentionally not thread-safe; one orchestration thread
 * performs start/poll/stop. All deadlines use `std::chrono::steady_clock`.
 */
class ScopedProcess
{
public:
  explicit ScopedProcess(
    std::string label = "дочерний процесс",
    ScopedProcessStopPolicy stop_policy = {})
  : label_(std::move(label)),
    stop_policy_(normalizeStopPolicy(stop_policy))
  {
  }

  ScopedProcess(const ScopedProcess &) = delete;
  ScopedProcess & operator=(const ScopedProcess &) = delete;

  ~ScopedProcess()
  {
    stop();
  }

  /// Spawn a non-empty command in its own process group.
  ///
  /// Repeated start while running is idempotent. Spawn failure leaves this
  /// object without child ownership and returns false.
  bool start(const std::vector<std::string> & command, const rclcpp::Logger & logger)
  {
    if (command.empty()) {
      RCLCPP_ERROR(logger, "Пустая команда дочернего процесса");
      return false;
    }
    if (isRunning()) {
      RCLCPP_WARN(logger, "%s уже запущен: pid=%d", label_.c_str(), pid_);
      return true;
    }

    std::vector<char *> argv;
    argv.reserve(command.size() + 1);
    for (const auto & argument : command) {
      argv.push_back(const_cast<char *>(argument.c_str()));
    }
    argv.push_back(nullptr);

    posix_spawnattr_t attr;
    if (posix_spawnattr_init(&attr) != 0) {
      RCLCPP_ERROR(logger, "posix_spawnattr_init() не удался");
      return false;
    }
    const int attributes_result = detail::configureIsolatedSpawnAttributes(&attr);
    if (attributes_result != 0) {
      (void)posix_spawnattr_destroy(&attr);
      RCLCPP_ERROR(
        logger,
        "Не удалось настроить отдельную process group: %s",
        std::strerror(attributes_result));
      return false;
    }

    pid_t pid = -1;
    const int spawn_result = posix_spawnp(&pid, argv[0], nullptr, &attr, argv.data(), environ);
    (void)posix_spawnattr_destroy(&attr);
    if (spawn_result != 0) {
      RCLCPP_ERROR(logger, "posix_spawnp() не удался: %s", std::strerror(spawn_result));
      return false;
    }

    pid_ = pid;
    wait_status_.reset();
    RCLCPP_INFO(
      logger,
      "Запущен %s: pid=%d, cmd=%s",
      label_.c_str(),
      pid_,
      joinCommand(command).c_str());
    return true;
  }

  /// Poll and reap a terminal child; return whether it is still running.
  bool isRunning()
  {
    if (pid_ <= 0) {
      return false;
    }

    int status = 0;
    const pid_t result = detail::waitPidRetryingInterrupt(pid_, &status, WNOHANG);
    if (result == 0) {
      return true;
    }
    if (result == pid_) {
      recordExitStatus(status);
      return false;
    }
    if (errno == ECHILD) {
      pid_ = -1;
      return false;
    }
    return true;
  }

  /// Reap a natural exit before a steady timeout without sending signals.
  bool waitForExit(std::chrono::milliseconds timeout)
  {
    if (timeout < std::chrono::milliseconds::zero()) {
      timeout = std::chrono::milliseconds::zero();
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (isRunning() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !isRunning();
  }

  /// Return the code only after this owner reaped a normal process exit.
  [[nodiscard]] std::optional<int> exitCode() const noexcept
  {
    if (!wait_status_.has_value() || !WIFEXITED(*wait_status_)) {
      return std::nullopt;
    }
    return WEXITSTATUS(*wait_status_);
  }

  /// Return true only for a reaped normal exit with code zero.
  [[nodiscard]] bool exitedSuccessfully() const noexcept
  {
    const auto code = exitCode();
    return code.has_value() && *code == 0;
  }

  /// Stop and reap the process group using SIGINT/SIGTERM/SIGKILL escalation.
  ///
  /// The cooperative grace windows are bounded. After SIGKILL the method waits
  /// for the mandatory child reap so ownership is never silently discarded.
  /// Repeated calls are harmless; OS signaling errors are handled best-effort.
  void stop()
  {
    if (pid_ <= 0) {
      return;
    }

    int status = 0;
    const pid_t result = detail::waitPidRetryingInterrupt(pid_, &status, WNOHANG);
    if (result == pid_) {
      recordExitStatus(status);
      return;
    }
    if (result < 0 && errno == ECHILD) {
      pid_ = -1;
      return;
    }

    terminateWith(SIGINT, stop_policy_.interrupt_grace);
    if (pid_ > 0) {
      terminateWith(SIGTERM, stop_policy_.terminate_grace);
    }
    if (pid_ > 0) {
      signalProcessGroupOrChild(SIGKILL);
      const pid_t wait_result = detail::waitPidRetryingInterrupt(pid_, &status, 0);
      if (wait_result == pid_) {
        recordExitStatus(status);
      } else if (wait_result < 0 && errno == ECHILD) {
        pid_ = -1;
      }
    }
  }

private:
  void terminateWith(int signal_number, std::chrono::milliseconds timeout)
  {
    if (pid_ <= 0) {
      return;
    }

    signalProcessGroupOrChild(signal_number);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      const pid_t result = detail::waitPidRetryingInterrupt(pid_, &status, WNOHANG);
      if (result == pid_) {
        recordExitStatus(status);
        return;
      }
      if (result < 0 && errno == ECHILD) {
        pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(stop_policy_.poll_interval);
    }
  }

  void signalProcessGroupOrChild(const int signal_number) const noexcept
  {
    const int group_result = kill(-pid_, signal_number);
    const pid_t current_group = getpgid(pid_);
    if (group_result == 0 && current_group == pid_) {
      return;
    }
    // A child can move itself to another process group. Retain direct-child
    // ownership even when group delivery is no longer possible.
    (void)kill(pid_, signal_number);
  }

  static ScopedProcessStopPolicy normalizeStopPolicy(ScopedProcessStopPolicy policy)
  {
    policy.interrupt_grace = std::max(
      policy.interrupt_grace, std::chrono::milliseconds::zero());
    policy.terminate_grace = std::max(
      policy.terminate_grace, std::chrono::milliseconds::zero());
    policy.poll_interval = std::max(
      policy.poll_interval, std::chrono::milliseconds(1));
    return policy;
  }

  void recordExitStatus(const int status) noexcept
  {
    wait_status_ = status;
    pid_ = -1;
  }

  std::string label_;
  ScopedProcessStopPolicy stop_policy_;
  pid_t pid_{-1};
  std::optional<int> wait_status_;
};
}  // namespace franka_motion
