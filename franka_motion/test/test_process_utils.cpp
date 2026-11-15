// Responsibility: Verifies process-group startup, exit evidence, timeout, and termination.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "franka_motion/process_utils.hpp"

using namespace std::chrono_literals;

TEST(ProcessUtils, RetriesWaitpidAfterInterrupt)
{
  int calls = 0;
  int status = 0;
  const pid_t expected_pid = 123;

  const pid_t result = franka_motion::detail::waitPidRetryingInterrupt(
    expected_pid,
    &status,
    WNOHANG,
    [&calls, expected_pid](pid_t, int *, int) {
      ++calls;
      if (calls == 1) {
        errno = EINTR;
        return static_cast<pid_t>(-1);
      }
      return expected_pid;
    });

  EXPECT_EQ(result, expected_pid);
  EXPECT_EQ(calls, 2);
}

TEST(ProcessUtils, RejectsIncompleteProcessGroupAttributes)
{
  posix_spawnattr_t attributes;
  ASSERT_EQ(posix_spawnattr_init(&attributes), 0);

  int process_group_calls = 0;
  const int flags_error = franka_motion::detail::configureIsolatedSpawnAttributes(
    &attributes,
    [](posix_spawnattr_t *, auto) {return EINVAL;},
    [&process_group_calls](posix_spawnattr_t *, pid_t) {
      ++process_group_calls;
      return 0;
    });
  EXPECT_EQ(flags_error, EINVAL);
  EXPECT_EQ(process_group_calls, 0);

  const int process_group_error = franka_motion::detail::configureIsolatedSpawnAttributes(
    &attributes,
    [](posix_spawnattr_t *, auto) {return 0;},
    [&process_group_calls](posix_spawnattr_t *, pid_t) {
      ++process_group_calls;
      return EPERM;
    });
  EXPECT_EQ(process_group_error, EPERM);
  EXPECT_EQ(process_group_calls, 1);
  EXPECT_EQ(posix_spawnattr_destroy(&attributes), 0);
}

TEST(ProcessUtils, StartsAndStopsProcessGroup)
{
  franka_motion::ScopedProcess process{"test sleep process"};

  ASSERT_TRUE(
    process.start(
      {"/bin/sleep", "5"},
      rclcpp::get_logger("test_process_utils")));
  EXPECT_TRUE(process.isRunning());

  process.stop();

  EXPECT_FALSE(process.isRunning());
}

TEST(ProcessUtils, WaitsForNaturalExitWithoutSendingSignals)
{
  franka_motion::ScopedProcess process{"test natural exit"};
  ASSERT_TRUE(
    process.start(
      {"/bin/true"},
      rclcpp::get_logger("test_process_utils")));

  EXPECT_TRUE(process.waitForExit(1s));
  EXPECT_FALSE(process.isRunning());
  ASSERT_TRUE(process.exitCode().has_value());
  EXPECT_EQ(*process.exitCode(), 0);
  EXPECT_TRUE(process.exitedSuccessfully());
}

TEST(ProcessUtils, PreservesNonZeroChildExitStatus)
{
  franka_motion::ScopedProcess process{"test failed child"};
  ASSERT_TRUE(
    process.start(
      {"/bin/false"},
      rclcpp::get_logger("test_process_utils")));

  ASSERT_TRUE(process.waitForExit(1s));
  ASSERT_TRUE(process.exitCode().has_value());
  EXPECT_NE(*process.exitCode(), 0);
  EXPECT_FALSE(process.exitedSuccessfully());
}

TEST(ProcessUtils, ReportsTimeoutWhileChildIsStillRunning)
{
  franka_motion::ScopedProcess process{"test wait timeout"};
  ASSERT_TRUE(
    process.start(
      {"/bin/sleep", "5"},
      rclcpp::get_logger("test_process_utils")));

  EXPECT_FALSE(process.waitForExit(10ms));
  EXPECT_TRUE(process.isRunning());
  EXPECT_FALSE(process.exitCode().has_value());
  EXPECT_FALSE(process.exitedSuccessfully());
  process.stop();
}

TEST(ProcessUtils, EscalatesToSigkillAndReapsSignalIgnoringChild)
{
  const auto ready_path =
    std::filesystem::temp_directory_path() /
    ("franka_process_utils_ready_" + std::to_string(getpid()));
  std::filesystem::remove(ready_path);
  franka_motion::ScopedProcessStopPolicy stop_policy;
  stop_policy.interrupt_grace = 20ms;
  stop_policy.terminate_grace = 20ms;
  stop_policy.poll_interval = 1ms;
  franka_motion::ScopedProcess process{"signal-ignoring test child", stop_policy};
  const std::vector<std::string> command{
    "/bin/sh", "-c",
    "trap '' INT TERM; printf ready > \"$1\"; while :; do sleep 1; done",
    "franka_process_utils_test", ready_path.string(),
  };
  ASSERT_TRUE(
    process.start(
      command,
      rclcpp::get_logger("test_process_utils")));

  const auto ready_deadline = std::chrono::steady_clock::now() + 1s;
  while (!std::filesystem::exists(ready_path) &&
    std::chrono::steady_clock::now() < ready_deadline)
  {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(std::filesystem::exists(ready_path));

  process.stop();

  EXPECT_FALSE(process.isRunning());
  EXPECT_FALSE(process.exitCode().has_value());
  std::filesystem::remove(ready_path);
}
