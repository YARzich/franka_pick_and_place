// Responsibility: Verifies cuRobo worker ownership, exit evidence, timeout, and cancellation.

#include <gtest/gtest.h>
#include <sys/prctl.h>
#include <sys/wait.h>

#include <chrono>
#include <cerrno>
#include <csignal>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "manipulation_curobo/child_process.hpp"
#include "manipulation_curobo/temporary_file.hpp"

namespace
{

using namespace std::chrono_literals;
using manipulation_curobo::curobo::ChildProcess;
using manipulation_curobo::curobo::ProcessOutcome;
using manipulation_curobo::curobo::TemporaryFile;

// Adopt only this test's orphaned descendant so that even a failing regression
// leaves no live process or zombie behind. No other process group is signaled.
class DescendantCleanup final
{
public:
  DescendantCleanup()
  {
    (void)::prctl(PR_GET_CHILD_SUBREAPER, &previous_subreaper_);
    enabled_ = ::prctl(PR_SET_CHILD_SUBREAPER, 1) == 0;
  }

  ~DescendantCleanup()
  {
    if (pid > 0) {
      (void)::kill(pid, SIGKILL);
      while (::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {}
    }
    if (enabled_) {
      (void)::prctl(PR_SET_CHILD_SUBREAPER, previous_subreaper_);
    }
  }

  bool enabled() const {return enabled_;}
  pid_t pid{-1};

private:
  int previous_subreaper_{0};
  bool enabled_{false};
};

constexpr const char * kWorkerWithDescendant =
  R"(
import os
import signal
import sys
import time

child = os.fork()
if child == 0:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    with open(sys.argv[1], 'w') as output:
        output.write(str(os.getpid()))
    while True:
        time.sleep(1)
while os.stat(sys.argv[1]).st_size == 0:
    time.sleep(0.005)
if sys.argv[2] == 'exit':
    raise SystemExit(7)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True:
    time.sleep(1)
)";

bool read_descendant_pid(const TemporaryFile & file, DescendantCleanup & cleanup)
{
  const auto deadline = ChildProcess::Clock::now() + 2s;
  while (ChildProcess::Clock::now() < deadline) {
    std::ifstream input(file.path());
    if (input >> cleanup.pid && cleanup.pid > 0) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

void expect_descendant_terminated(DescendantCleanup & cleanup)
{
  int status = 0;
  pid_t result = 0;
  const auto deadline = ChildProcess::Clock::now() + 1s;
  while (ChildProcess::Clock::now() < deadline) {
    result = ::waitpid(cleanup.pid, &status, WNOHANG);
    if (result != 0) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(result, cleanup.pid) << "owned worker descendant survived cleanup";
  if (result == cleanup.pid) {
    cleanup.pid = -1;
    EXPECT_TRUE(WIFSIGNALED(status));
    EXPECT_EQ(WTERMSIG(status), SIGKILL);
  }
}

TEST(ChildProcess, TimeoutEscalatesAfterLeaderExitsButDescendantIgnoresSignals)
{
  DescendantCleanup cleanup;
  ASSERT_TRUE(cleanup.enabled());
  TemporaryFile pid_file{"curobo_descendant_timeout"};
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c", kWorkerWithDescendant, pid_file.path().string(), "wait"}));
  ASSERT_TRUE(read_descendant_pid(pid_file, cleanup));

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 20ms);

  EXPECT_EQ(result.outcome, ProcessOutcome::kTimedOut);
  EXPECT_EQ(process.pid(), -1);
  expect_descendant_terminated(cleanup);
}

TEST(ChildProcess, NormalLeaderExitPreservesStatusAndStopsRemainingDescendants)
{
  DescendantCleanup cleanup;
  ASSERT_TRUE(cleanup.enabled());
  TemporaryFile pid_file{"curobo_descendant_exit"};
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c", kWorkerWithDescendant, pid_file.path().string(), "exit"}));
  ASSERT_TRUE(read_descendant_pid(pid_file, cleanup));

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 2s);

  EXPECT_EQ(result.outcome, ProcessOutcome::kExited);
  EXPECT_EQ(result.exit_code, 7);
  expect_descendant_terminated(cleanup);
}

TEST(ChildProcess, OwnerDestructionStopsDescendantAfterLeaderHasExited)
{
  DescendantCleanup cleanup;
  ASSERT_TRUE(cleanup.enabled());
  TemporaryFile pid_file{"curobo_descendant_shutdown"};
  {
    ChildProcess process;
    ASSERT_TRUE(process.start({
        "/usr/bin/python3", "-c", kWorkerWithDescendant, pid_file.path().string(), "wait"}));
    ASSERT_TRUE(read_descendant_pid(pid_file, cleanup));
    process.requestStop();
  }
  expect_descendant_terminated(cleanup);
}

TEST(ChildProcess, ReturnsExactExitEvidenceWithoutShellParsing)
{
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c",
      "import sys; raise SystemExit(0 if sys.argv[1] == ';echo unsafe' else 9)",
      ";echo unsafe"}));

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 2s);

  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(result.outcome, ProcessOutcome::kExited);
  EXPECT_EQ(result.exit_code, 0);
  EXPECT_EQ(process.pid(), -1);
}

TEST(ChildProcess, ReportsNonZeroExitCode)
{
  ChildProcess process;
  ASSERT_TRUE(process.start({"/usr/bin/python3", "-c", "raise SystemExit(7)"}));

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 2s);

  EXPECT_EQ(result.outcome, ProcessOutcome::kExited);
  EXPECT_EQ(result.exit_code, 7);
  EXPECT_FALSE(result.succeeded());
}

TEST(ChildProcess, UnboundedWaitDoesNotChargeBackendStartupToPlanningBudget)
{
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c", "import time; time.sleep(0.08)"}));
  const auto started_at = ChildProcess::Clock::now();

  const auto result = process.wait();

  EXPECT_TRUE(result.succeeded());
  EXPECT_GE(ChildProcess::Clock::now() - started_at, 50ms);
  EXPECT_EQ(process.pid(), -1);
}

TEST(ChildProcess, TimeoutTerminatesAndReapsTheOwnedGroup)
{
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c", "import time; time.sleep(30)"}));
  const pid_t child_pid = process.pid();

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 20ms);

  EXPECT_EQ(result.outcome, ProcessOutcome::kTimedOut);
  EXPECT_EQ(process.pid(), -1);
  int status = 0;
  errno = 0;
  EXPECT_EQ(::waitpid(child_pid, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

TEST(ChildProcess, ConcurrentStopIsStickyAndGoalSpecific)
{
  ChildProcess process;
  ASSERT_TRUE(process.start({
      "/usr/bin/python3", "-c", "import time; time.sleep(30)"}));
  std::thread canceler([&process]() {
      process.requestStop();
    });

  const auto result = process.waitUntil(ChildProcess::Clock::now() + 2s);
  canceler.join();

  EXPECT_EQ(result.outcome, ProcessOutcome::kCanceled);
  EXPECT_EQ(process.pid(), -1);
}

TEST(ChildProcess, FailedSpawnReturnsTypedResult)
{
  ChildProcess process;
  EXPECT_FALSE(process.start({"/definitely/not/a/program"}));

  const auto result = process.waitUntil(ChildProcess::Clock::now());

  EXPECT_EQ(result.outcome, ProcessOutcome::kSpawnFailed);
  EXPECT_FALSE(result.detail.empty());
}

TEST(ChildProcess, CancelBeforeSpawnSuppressesThePhysicalSideEffect)
{
  ChildProcess process;
  process.requestStop();

  EXPECT_FALSE(process.start({"/usr/bin/python3", "-c", "raise SystemExit(0)"}));
  const auto result = process.waitUntil(ChildProcess::Clock::now() + 1s);

  EXPECT_EQ(result.outcome, ProcessOutcome::kCanceled);
  EXPECT_EQ(process.pid(), -1);
}

TEST(ChildProcess, OwnerShutdownTerminatesAndReapsItsExactChild)
{
  pid_t child_pid = -1;
  {
    ChildProcess process;
    ASSERT_TRUE(process.start({
        "/usr/bin/python3", "-c", "import time; time.sleep(30)"}));
    child_pid = process.pid();
  }

  int status = 0;
  errno = 0;
  EXPECT_EQ(::waitpid(child_pid, &status, WNOHANG), -1);
  EXPECT_EQ(errno, ECHILD);
}

}  // namespace
