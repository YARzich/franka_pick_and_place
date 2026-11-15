// Responsibility: Verifies ServoToPose success, cancel, pause, failures, and deadlines in memory.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

#include "servo_to_pose_task_test_support.hpp"

namespace
{

using manipulation::moveit::ServoToPoseTask;
using manipulation::moveit::ServoToPoseTaskControl;
using manipulation::moveit::ServoToPoseTaskFeedback;
using manipulation::moveit::test::FakeServoToPosePort;
using manipulation::moveit::test::fast_task_config;
using manipulation::moveit::test::valid_target;
using manipulation::moveit::test::wait_until;
using manipulation::task::TaskErrorCode;
using namespace std::chrono_literals;

TEST(ServoToPoseTask, ReachesTargetAndConfirmsStoppedBackend)
{
  FakeServoToPosePort port;
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_TRUE(result.succeeded());
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(port.configure_calls, 1U);
  EXPECT_EQ(port.prepare_calls, 1U);
  EXPECT_GE(port.pause_calls, 2U);
  EXPECT_GE(port.publish_calls, 1U);
}

TEST(ServoToPoseTask, RejectsFinalStoppedPoseOutsideTolerance)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kFinalPoseOutsideTolerance);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kExecutionFailed);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_NE(result.detail.find("outside tolerance"), std::string::npos);
  EXPECT_GE(port.pause_calls, 2U);
}

TEST(ServoToPoseTask, MapsFinalStoppedPoseEvidenceFailures)
{
  FakeServoToPosePort stale_port(FakeServoToPosePort::Mode::kFinalPoseStale);
  ServoToPoseTask stale_task(stale_port, fast_task_config());
  ServoToPoseTaskControl stale_control;
  auto start = std::chrono::steady_clock::now();

  auto result = stale_task.run(
    valid_target(), start, start + 200ms, stale_control,
    [](const ServoToPoseTaskFeedback &) {});
  EXPECT_EQ(result.error_code, TaskErrorCode::kRobotStateStale);
  EXPECT_TRUE(result.stop_confirmed);

  FakeServoToPosePort invalid_port(FakeServoToPosePort::Mode::kFinalPoseInvalid);
  ServoToPoseTask invalid_task(invalid_port, fast_task_config());
  ServoToPoseTaskControl invalid_control;
  start = std::chrono::steady_clock::now();
  result = invalid_task.run(
    valid_target(), start, start + 200ms, invalid_control,
    [](const ServoToPoseTaskFeedback &) {});
  EXPECT_EQ(result.error_code, TaskErrorCode::kTfError);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTask, CancelStopsPublicationBeforeReturningCanceled)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(std::launch::async, [&]() {
        return task.run(
        valid_target(), start, start + 300ms, control,
          [](const ServoToPoseTaskFeedback &) {});
    });
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 2U;}));

  control.request_cancel();
  const auto result = future.get();

  EXPECT_EQ(result.error_code, TaskErrorCode::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_GE(port.pause_calls, 2U);
}

TEST(ServoToPoseTask, PauseHoldsTargetStreamUntilExplicitResume)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  port.suppress_status_while_paused = true;
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  std::mutex feedback_mutex;
  std::vector<ServoToPoseTaskFeedback> feedback;
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(std::launch::async, [&]() {
        return task.run(
        valid_target(), start, start + 500ms, control,
          [&](const ServoToPoseTaskFeedback & sample) {
            const std::lock_guard<std::mutex> lock(feedback_mutex);
            feedback.push_back(sample);
        });
    });
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 2U;}));

  control.request_pause();
  ASSERT_TRUE(wait_until([&]() {return port.pause_calls.load() >= 2U;}));
  const auto publishes_while_paused = port.publish_calls.load();
  EXPECT_EQ(future.wait_for(30ms), std::future_status::timeout);
  EXPECT_EQ(port.publish_calls.load(), publishes_while_paused);

  control.resume();
  ASSERT_TRUE(wait_until([&]() {return port.unpause_calls.load() >= 2U;}));
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() > publishes_while_paused;}));
  control.request_cancel();
  EXPECT_EQ(future.get().error_code, TaskErrorCode::kCanceled);

  const std::lock_guard<std::mutex> lock(feedback_mutex);
  EXPECT_TRUE(std::any_of(feedback.begin(), feedback.end(), [](const auto & sample) {
      return sample.state.paused;
    }));
}

TEST(ServoToPoseTask, ResumeRequiresFreshStatusBeforePublishingAgain)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  std::atomic_bool safely_paused{false};
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(std::launch::async, [&]() {
        return task.run(
        valid_target(), start, start + 300ms, control,
          [&](const ServoToPoseTaskFeedback & feedback) {
            if (feedback.state.paused) {
              safely_paused = true;
            }
          });
    });
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 2U;}));

  control.request_pause();
  ASSERT_TRUE(wait_until([&]() {return safely_paused.load();}));
  const auto publications_before_resume = port.publish_calls.load();
  port.suppress_status_feedback = true;
  control.resume();

  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto result = future.get();
  EXPECT_EQ(result.error_code, TaskErrorCode::kRobotStateStale);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_GE(port.unpause_calls.load(), 2U);
  EXPECT_EQ(port.publish_calls.load(), publications_before_resume);
}

TEST(ServoToPoseTask, StalePoseUsesConfirmedFailClosedHalt)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kStaleAfterPublish);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kRobotStateStale);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_GE(port.pause_calls, 2U);
}

TEST(ServoToPoseTask, SevereServoStatusUsesConfirmedFailClosedHalt)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kSafetyStopAfterPublish);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kExecutionFailed);
  EXPECT_TRUE(result.stop_confirmed);
}

TEST(ServoToPoseTask, AbsoluteTimeoutIncludesItsSafeStopBudget)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  auto config = fast_task_config();
  config.state_machine.halt_timeout = 10ms;
  ServoToPoseTask task(port, config);
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 40ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kTimeout);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_LE(result.elapsed, 40ms);
}

TEST(ServoToPoseTask, SetupDeadlineStillUsesReservedSafeStopBudget)
{
  FakeServoToPosePort port;
  port.block_prepare_until_deadline = true;
  auto config = fast_task_config();
  config.state_machine.halt_timeout = 15ms;
  config.target_prepare_timeout = 100ms;
  ServoToPoseTask task(port, config);
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 45ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kTimeout);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(port.prepare_calls, 1U);
  EXPECT_EQ(port.unpause_calls, 0U);
  EXPECT_EQ(port.publish_calls, 0U);
  EXPECT_LE(result.elapsed, 45ms);
}

TEST(ServoToPoseTask, CancelBeforeFirstResumeNeverUnpausesBackend)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  control.request_cancel();
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kCanceled);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(port.unpause_calls, 0U);
  EXPECT_EQ(port.publish_calls, 0U);
}

TEST(ServoToPoseTask, PauseBeforeFirstResumeKeepsBackendPausedUntilResume)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  control.request_pause();
  std::atomic_bool safely_paused{false};
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(std::launch::async, [&]() {
        return task.run(
        valid_target(), start, start + 300ms, control,
          [&](const ServoToPoseTaskFeedback & feedback) {
            if (feedback.state.paused) {
              safely_paused = true;
            }
        });
    });

  ASSERT_TRUE(wait_until([&]() {return safely_paused.load();}));
  EXPECT_EQ(port.unpause_calls, 0U);
  EXPECT_EQ(port.publish_calls, 0U);

  control.resume();
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 1U;}));
  control.request_cancel();
  EXPECT_EQ(future.get().error_code, TaskErrorCode::kCanceled);
}

TEST(ServoToPoseTask, WaitsForCoherentStartupFeedbackBeforeFirstPublication)
{
  FakeServoToPosePort port(FakeServoToPosePort::Mode::kNeverReach);
  port.readiness_after_observations = 4U;
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();
  auto future = std::async(std::launch::async, [&]() {
        return task.run(
        valid_target(), start, start + 300ms, control,
          [](const ServoToPoseTaskFeedback &) {});
    });

  std::this_thread::sleep_for(2ms);
  EXPECT_EQ(port.publish_calls, 0U);
  ASSERT_TRUE(wait_until([&]() {return port.publish_calls.load() >= 1U;}));
  control.request_cancel();
  EXPECT_EQ(future.get().error_code, TaskErrorCode::kCanceled);
}

TEST(ServoToPoseTask, PreparationFailureStillConfirmsStopBeforeReturning)
{
  FakeServoToPosePort port;
  port.fail_prepare = true;
  ServoToPoseTask task(port, fast_task_config());
  ServoToPoseTaskControl control;
  const auto start = std::chrono::steady_clock::now();

  const auto result = task.run(
    valid_target(), start, start + 200ms, control,
    [](const ServoToPoseTaskFeedback &) {});

  EXPECT_EQ(result.error_code, TaskErrorCode::kTfError);
  EXPECT_TRUE(result.stop_confirmed);
  EXPECT_EQ(port.unpause_calls, 0U);
  EXPECT_EQ(port.publish_calls, 0U);
}

}  // namespace
