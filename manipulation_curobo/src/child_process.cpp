// Responsibility: Spawns, polls, signals and reaps the external planner without leaking child
//   processes.

#include "manipulation_curobo/child_process.hpp"

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

extern char ** environ;

namespace manipulation_curobo::curobo
{
namespace
{

using namespace std::chrono_literals;
constexpr auto kWaitQuantum = 5ms;
constexpr auto kInterruptGrace = 250ms;
constexpr auto kTerminateGrace = 250ms;

ProcessResult decodeStatus(const siginfo_t & status)
{
  if (status.si_code == CLD_EXITED) {
    const int exit_code = status.si_status;
    return ProcessResult{
      ProcessOutcome::kExited, exit_code, 0,
      exit_code == 0 ? "child exited successfully" : "child returned a non-zero exit code"};
  }
  if (status.si_code == CLD_KILLED || status.si_code == CLD_DUMPED) {
    return ProcessResult{
      ProcessOutcome::kSignaled, -1, status.si_status, "child terminated by a signal"};
  }
  return ProcessResult{
    ProcessOutcome::kWaitFailed, -1, 0, "child produced an unsupported wait status"};
}

class SpawnAttributes final
{
public:
  SpawnAttributes()
  : initialized_(::posix_spawnattr_init(&value_) == 0)
  {
  }

  ~SpawnAttributes()
  {
    if (initialized_) {
      (void)::posix_spawnattr_destroy(&value_);
    }
  }

  SpawnAttributes(const SpawnAttributes &) = delete;
  SpawnAttributes & operator=(const SpawnAttributes &) = delete;

  [[nodiscard]] bool configureProcessGroup()
  {
    return initialized_ &&
           ::posix_spawnattr_setflags(&value_, POSIX_SPAWN_SETPGROUP) == 0 &&
           ::posix_spawnattr_setpgroup(&value_, 0) == 0;
  }

  [[nodiscard]] posix_spawnattr_t * get() noexcept
  {
    return initialized_ ? &value_ : nullptr;
  }

private:
  posix_spawnattr_t value_{};
  bool initialized_{false};
};

}  // namespace

bool ProcessResult::succeeded() const noexcept
{
  return outcome == ProcessOutcome::kExited && exit_code == 0;
}

ChildProcess::~ChildProcess() noexcept
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (pid_ <= 0) {
    return;
  }
  lock.unlock();
  (void)stopAndReap(ProcessOutcome::kCanceled, "child stopped during owner destruction");
}

bool ChildProcess::start(const std::vector<std::string> & argv)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_ || stop_requested_ || argv.empty() || argv.front().empty()) {
    return false;
  }
  for (const auto & argument : argv) {
    if (argument.find('\0') != std::string::npos) {
      return false;
    }
  }
  started_ = true;

  SpawnAttributes attributes;
  if (!attributes.configureProcessGroup()) {
    spawn_error_ = EINVAL;
    return false;
  }

  std::vector<char *> raw_argv;
  raw_argv.reserve(argv.size() + 1U);
  for (const auto & argument : argv) {
    raw_argv.push_back(const_cast<char *>(argument.c_str()));
  }
  raw_argv.push_back(nullptr);

  pid_t child_pid = -1;
  spawn_error_ = ::posix_spawnp(
    &child_pid, raw_argv.front(), nullptr, attributes.get(), raw_argv.data(), environ);
  if (spawn_error_ != 0) {
    return false;
  }
  pid_ = child_pid;
  return true;
}

ProcessResult ChildProcess::waitUntil(const Clock::time_point deadline)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pid_ <= 0) {
      if (stop_requested_) {
        return ProcessResult{
          ProcessOutcome::kCanceled, -1, 0,
          "child start suppressed by an earlier cancellation"};
      }
      if (spawn_error_ != 0) {
        return ProcessResult{
          ProcessOutcome::kSpawnFailed, -1, 0,
          std::string{"failed to spawn child: "} + std::strerror(spawn_error_)};
      }
      return ProcessResult{
        ProcessOutcome::kWaitFailed, -1, 0, "wait requested before a successful start"};
    }
  }

  while (true) {
    std::optional<ProcessResult> completed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto result = pollExitLocked();
      if (result.outcome != ProcessOutcome::kWaitFailed || pid_ <= 0) {
        if (stop_requested_ && result.outcome != ProcessOutcome::kWaitFailed) {
          result.outcome = ProcessOutcome::kCanceled;
          result.detail = "child canceled by its owner";
        }
        completed = std::move(result);
      }
      if (!completed && stop_requested_) {
        // requestStop() already sent SIGINT; escalation and reap stay here.
        break;
      }
    }
    if (completed) {
      // The waitable leader pins its PID/PGID until the last cleanup signal.
      // Keep its original result while stopping any remaining descendants.
      (void)stopAndReap(completed->outcome, completed->detail);
      return *completed;
    }

    const auto now = Clock::now();
    if (now >= deadline) {
      return stopAndReap(ProcessOutcome::kTimedOut, "child exceeded steady deadline");
    }
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_until(lock, std::min(deadline, now + kWaitQuantum), [this]() {
        return stop_requested_;
    });
  }
  return stopAndReap(ProcessOutcome::kCanceled, "child canceled by its owner");
}

ProcessResult ChildProcess::wait()
{
  return waitUntil(Clock::time_point::max());
}

void ChildProcess::requestStop() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  stop_requested_ = true;
  signalGroupLocked(SIGINT);
  wake_.notify_all();
}

pid_t ChildProcess::pid() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pid_;
}

ProcessResult ChildProcess::pollExitLocked()
{
  if (pid_ <= 0) {
    return ProcessResult{
      ProcessOutcome::kWaitFailed, -1, 0, "child is not running"};
  }
  siginfo_t status{};
  const int result = ::waitid(
    P_PID, static_cast<id_t>(pid_), &status, WEXITED | WNOHANG | WNOWAIT);
  if (result == 0 && status.si_pid == 0) {
    return ProcessResult{
      ProcessOutcome::kWaitFailed, -1, 0, "child is still running"};
  }
  if (result == 0) {
    return decodeStatus(status);
  }
  if (result < 0 && errno == EINTR) {
    return ProcessResult{
      ProcessOutcome::kWaitFailed, -1, 0, "wait interrupted while child is running"};
  }
  const int wait_error = errno;
  pid_ = -1;
  return ProcessResult{
    ProcessOutcome::kWaitFailed, -1, 0,
    std::string{"waitid failed: "} + std::strerror(wait_error)};
}

ProcessResult ChildProcess::stopAndReap(
  const ProcessOutcome outcome, std::string detail) noexcept
{
  int status = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pid_ <= 0) {
      return ProcessResult{outcome, -1, 0, std::move(detail)};
    }
    signalGroupLocked(SIGINT);
  }
  if (!waitForExitUntil(Clock::now() + kInterruptGrace)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      signalGroupLocked(SIGTERM);
    }
    (void)waitForExitUntil(Clock::now() + kTerminateGrace);
  }
  // Even a terminal leader may have left signal-resistant helpers. WNOWAIT
  // keeps its numeric identity reserved until this final group signal.
  pid_t child_pid = -1;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    child_pid = pid_;
    signalGroupLocked(SIGKILL);
    // requestStop() must not signal this numeric group after waitpid releases
    // its identity. Close signaling before the blocking reap, under its mutex.
    group_signals_closed_ = true;
  }
  if (child_pid > 0) {
    while (::waitpid(child_pid, &status, 0) < 0 && errno == EINTR) {}
  }
  // Only the direct child can be reaped here; orphan zombies belong to the
  // system reaper and cannot continue executing or holding CUDA resources.
  std::lock_guard<std::mutex> lock(mutex_);
  resetLocked();
  return ProcessResult{outcome, -1, 0, std::move(detail)};
}

void ChildProcess::signalGroupLocked(const int signal_number) noexcept
{
  if (pid_ > 0 && !group_signals_closed_) {
    (void)::kill(-pid_, signal_number);
  }
}

bool ChildProcess::waitForExitUntil(const Clock::time_point deadline) noexcept
{
  while (Clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto result = pollExitLocked();
      if (result.outcome != ProcessOutcome::kWaitFailed || pid_ <= 0) {
        return true;
      }
    }
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait_until(lock, std::min(deadline, Clock::now() + kWaitQuantum));
  }
  return false;
}

void ChildProcess::resetLocked() noexcept
{
  pid_ = -1;
}

}  // namespace manipulation_curobo::curobo
