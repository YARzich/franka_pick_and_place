// Responsibility: Verify configured observations and interruptible detection waits.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "tf2_ros/buffer.hpp"

#include "workflow_v2/observation_providers.hpp"

namespace
{

using namespace std::chrono_literals;

class TestContext final : public manipulation::workflow::TransferExecutionContext
{
public:
  using CheckpointHook = std::function<void(std::size_t)>;
  using WaitHook = std::function<void(std::size_t)>;
  using WaitingHook =
    std::function<void(manipulation::workflow::TransferWaitingReason)>;

  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    if (checkpoint_hook) {
      const auto call = checkpoint_calls.fetch_add(1U, std::memory_order_acq_rel) + 1U;
      checkpoint_hook(call);
    }
    return stop_requested() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  bool stop_requested() const noexcept override
  {
    return stopped.load(std::memory_order_acquire);
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(
    const TimePoint deadline) override
  {
    if (wait_hook) {
      const auto now = Clock::now();
      if (deadline > now) {
        virtual_wait_nanoseconds.fetch_add(
          std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count(),
          std::memory_order_acq_rel);
      }
      const auto call = wait_calls.fetch_add(1U, std::memory_order_acq_rel) + 1U;
      wait_hook(call);
      return checkpoint();
    }
    while (!stop_requested() && Clock::now() < deadline) {
      std::this_thread::sleep_for(1ms);
    }
    return checkpoint();
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void report_waiting(
    const manipulation::workflow::TransferWaitingReason reason,
    manipulation::task::TaskStateSnapshot) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      waiting_reasons.push_back(reason);
    }
    if (waiting_hook) {
      waiting_hook(reason);
    }
  }

  manipulation::workflow::TaskStopCause stop_cause() const noexcept override
  {
    return stop_requested() ?
           stop_cause_value.load(std::memory_order_acquire) :
           manipulation::workflow::TaskStopCause::kNone;
  }

  void request_stop(const manipulation::workflow::TaskStopCause cause)
  {
    stop_cause_value.store(cause, std::memory_order_release);
    stopped.store(true, std::memory_order_release);
  }

  std::chrono::nanoseconds virtual_wait() const noexcept
  {
    return std::chrono::nanoseconds{
      virtual_wait_nanoseconds.load(std::memory_order_acquire)};
  }

  std::atomic<bool> stopped{false};
  mutable std::mutex mutex;
  std::vector<manipulation::workflow::TransferWaitingReason> waiting_reasons;
  CheckpointHook checkpoint_hook;
  WaitHook wait_hook;
  WaitingHook waiting_hook;

private:
  std::atomic<manipulation::workflow::TaskStopCause> stop_cause_value{
    manipulation::workflow::TaskStopCause::kClientCanceled};
  std::atomic<std::size_t> checkpoint_calls{0U};
  std::atomic<std::size_t> wait_calls{0U};
  std::atomic<std::int64_t> virtual_wait_nanoseconds{0};
};

geometry_msgs::msg::Pose valid_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.4;
  pose.orientation.w = 1.0;
  return pose;
}

manipulation::workflow::ObjectProfile profile()
{
  geometry_msgs::msg::Pose local;
  local.orientation.w = 1.0;
  return manipulation::workflow::ObjectProfile::create(
    "cube_profile",
    manipulation::workflow::ObjectGeometry::primitive(
      manipulation::workflow::PrimitiveGeometry::create(
        manipulation::workflow::PrimitiveGeometryType::kBox,
        {0.05, 0.05, 0.05}, local)));
}

manipulation::workflow::ObjectInstance instance()
{
  return manipulation::workflow::ObjectInstance::create("cube", "cube_profile");
}

geometry_msgs::msg::PoseStamped detected_pose(
  const int second, const std::string & frame_id = "panda_link0")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp.sec = second;
  pose.pose = valid_pose();
  return pose;
}

geometry_msgs::msg::TransformStamped exact_transform(
  const int second, const std::string & target_frame, const std::string & source_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = target_frame;
  transform.header.stamp.sec = second;
  transform.child_frame_id = source_frame;
  transform.transform.translation.x = 1.0;
  transform.transform.rotation.w = 1.0;
  return transform;
}

bool saw_waiting_reason(
  const TestContext & context,
  const manipulation::workflow::TransferWaitingReason reason)
{
  std::lock_guard<std::mutex> lock(context.mutex);
  return std::find(
    context.waiting_reasons.cbegin(), context.waiting_reasons.cend(), reason) !=
         context.waiting_reasons.cend();
}

void expect_exact_tf_wait_stops(
  const manipulation::workflow::TaskStopCause stop_cause)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    []() {return detected_pose(62).header.stamp;});
  TestContext context;
  std::atomic<int> downstream_side_effects{0};
  bool observation_supplied = false;
  context.waiting_hook = [&buffer, &observation_supplied](
    const manipulation::workflow::TransferWaitingReason reason)
    {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation &&
        !observation_supplied)
      {
        observation_supplied = true;
        buffer.updateCubePose(detected_pose(61, "camera"));
      }
    };
  context.wait_hook = [&context, stop_cause](const std::size_t call) {
      if (call == 3U) {
        context.request_stop(stop_cause);
      }
    };

  const auto started = std::chrono::steady_clock::now();
  const auto result = provider.observe(instance(), profile(), context);
  if (result.succeeded()) {
    downstream_side_effects.fetch_add(1, std::memory_order_acq_rel);
  }
  const auto wall_elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_EQ(result.error_code(), manipulation::task::TaskErrorCode::kCanceled);
  EXPECT_EQ(context.stop_cause(), stop_cause);
  EXPECT_TRUE(saw_waiting_reason(
      context, manipulation::workflow::TransferWaitingReason::kTransform));
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 0);
  EXPECT_LT(wall_elapsed, 1s);

  EXPECT_TRUE(tf_buffer->setTransform(
      exact_transform(61, "panda_link0", "camera"), "acceptance_test", false));
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 0);
}

}  // namespace

TEST(V2ConfiguredObservationProvider, StampsPresetAndAcceptsGoalAfterStopGeneration)
{
  franka_motion::workflow_v2::ConfiguredObservationPreset preset;
  preset.emplace(
    "cube", franka_motion::workflow_v2::ConfiguredObservation{
    "panda_link0", valid_pose(), 0.9});
  int next_stamp = 40;
  franka_motion::workflow_v2::ConfiguredObservationProvider provider(
    std::move(preset), [&next_stamp]() {
      builtin_interfaces::msg::Time stamp;
      stamp.sec = next_stamp++;
      return stamp;
    });
  TestContext first;
  const auto first_result = provider.observe(instance(), profile(), first);
  ASSERT_TRUE(first_result.succeeded());
  EXPECT_EQ(first_result.value()->pose().header.stamp.sec, 40);

  provider.request_stop();
  TestContext second;
  const auto second_result = provider.observe(instance(), profile(), second);
  ASSERT_TRUE(second_result.succeeded());
  EXPECT_EQ(second_result.value()->pose().header.stamp.sec, 41);
}

TEST(V2DetectedObservationProvider, CancelInterruptsWaitAndFollowingGoalStillSucceeds)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    []() {return detected_pose(53).header.stamp;});

  TestContext first;
  auto canceled = std::async(std::launch::async, [&]() {
        return provider.observe(instance(), profile(), first);
  });
  std::this_thread::sleep_for(120ms);
  provider.request_stop();
  ASSERT_EQ(canceled.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(canceled.get().error_code(), manipulation::task::TaskErrorCode::kCanceled);

  TestContext second;
  auto succeeded = std::async(std::launch::async, [&]() {
        return provider.observe(instance(), profile(), second);
  });
  std::this_thread::sleep_for(120ms);
  buffer.updateCubePose(detected_pose(52));
  const auto status = succeeded.wait_for(1s);
  if (status != std::future_status::ready) {
    provider.request_stop();
    static_cast<void>(succeeded.wait_for(1s));
  }
  ASSERT_EQ(status, std::future_status::ready);
  const auto result = succeeded.get();
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->object_id(), "cube");
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 52);
  EXPECT_EQ(result.value()->pose().header.frame_id, "panda_link0");
  ASSERT_FALSE(second.waiting_reasons.empty());
  EXPECT_EQ(
    second.waiting_reasons.front(),
    manipulation::workflow::TransferWaitingReason::kObservation);
}

TEST(V2DetectedObservationProvider, ObservationContinuesAfterRemovedOneHundredEightySecondLimit)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    []() {return detected_pose(60).header.stamp;});
  TestContext context;
  std::atomic<int> virtual_elapsed_seconds{0};
  std::atomic<int> downstream_side_effects{0};
  context.checkpoint_hook = [&buffer, &virtual_elapsed_seconds](const std::size_t) {
      constexpr int kVirtualObservationSliceSeconds = 60;
      const int elapsed =
        virtual_elapsed_seconds.fetch_add(
        kVirtualObservationSliceSeconds, std::memory_order_acq_rel) +
        kVirtualObservationSliceSeconds;
      if (elapsed > 180) {
        buffer.updateCubePose(detected_pose(59));
      }
    };

  const auto started = std::chrono::steady_clock::now();
  const auto result = provider.observe(instance(), profile(), context);
  if (result.succeeded()) {
    downstream_side_effects.fetch_add(1, std::memory_order_acq_rel);
  }
  const auto wall_elapsed = std::chrono::steady_clock::now() - started;

  ASSERT_TRUE(result.succeeded());
  EXPECT_GT(virtual_elapsed_seconds.load(std::memory_order_acquire), 180);
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 59);
  EXPECT_EQ(downstream_side_effects.load(std::memory_order_acquire), 1);
  EXPECT_LT(wall_elapsed, 1s);
}

TEST(V2DetectedObservationProvider, LiftsSunkDetectionToConfiguredSupportBound)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0", 0.027},
    []() {return detected_pose(65).header.stamp;});
  TestContext context;
  bool observation_supplied = false;
  context.waiting_hook = [&buffer, &observation_supplied](
    const manipulation::workflow::TransferWaitingReason reason)
    {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation &&
        !observation_supplied)
      {
        observation_supplied = true;
        auto sunk = detected_pose(61);
        sunk.pose.position.z = 0.0248;
        buffer.updateCubePose(sunk);
      }
    };

  const auto result = provider.observe(instance(), profile(), context);

  ASSERT_TRUE(result.succeeded());
  EXPECT_DOUBLE_EQ(result.value()->pose().pose.position.z, 0.027);
}

TEST(V2DetectedObservationProvider, KeepsDetectionAboveSupportBoundUnchanged)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0", 0.027},
    []() {return detected_pose(65).header.stamp;});
  TestContext context;
  bool observation_supplied = false;
  context.waiting_hook = [&buffer, &observation_supplied](
    const manipulation::workflow::TransferWaitingReason reason)
    {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation &&
        !observation_supplied)
      {
        observation_supplied = true;
        auto resting = detected_pose(62);
        resting.pose.position.z = 0.031;
        buffer.updateCubePose(resting);
      }
    };

  const auto result = provider.observe(instance(), profile(), context);

  ASSERT_TRUE(result.succeeded());
  EXPECT_DOUBLE_EQ(result.value()->pose().pose.position.z, 0.031);
}

TEST(V2DetectedObservationProvider, ExactStampTfContinuesAfterRemovedTwoSecondLimit)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    []() {return detected_pose(65).header.stamp;});
  TestContext context;
  bool observation_supplied = false;
  bool transform_inserted = false;
  context.waiting_hook = [&buffer, &observation_supplied](
    const manipulation::workflow::TransferWaitingReason reason)
    {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation &&
        !observation_supplied)
      {
        observation_supplied = true;
        buffer.updateCubePose(detected_pose(60, "camera"));
      }
    };
  context.wait_hook = [&tf_buffer, &transform_inserted](const std::size_t call) {
      if (call == 50U) {
        transform_inserted = tf_buffer->setTransform(
          exact_transform(60, "panda_link0", "camera"), "acceptance_test", false);
      }
    };

  const auto started = std::chrono::steady_clock::now();
  const auto result = provider.observe(instance(), profile(), context);
  const auto wall_elapsed = std::chrono::steady_clock::now() - started;

  ASSERT_TRUE(result.succeeded());
  EXPECT_TRUE(transform_inserted);
  EXPECT_GT(context.virtual_wait(), 2s);
  EXPECT_LT(wall_elapsed, 1s);
  EXPECT_TRUE(saw_waiting_reason(
      context, manipulation::workflow::TransferWaitingReason::kTransform));
  EXPECT_EQ(result.value()->pose().header.frame_id, "panda_link0");
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 60);
  EXPECT_NEAR(result.value()->pose().pose.position.x, 1.4, 1.0e-9);
}

TEST(V2DetectedObservationProvider, ClientCancelDuringExactStampTfHasNoLaterSideEffect)
{
  expect_exact_tf_wait_stops(manipulation::workflow::TaskStopCause::kClientCanceled);
}

TEST(V2DetectedObservationProvider, ShutdownDuringExactStampTfHasNoLaterSideEffect)
{
  expect_exact_tf_wait_stops(manipulation::workflow::TaskStopCause::kShutdown);
}

TEST(V2DetectedObservationProvider, RejectsOldFutureAndMalformedSamplesWithoutConsumingThem)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    []() {return detected_pose(100).header.stamp;});
  std::vector<geometry_msgs::msg::PoseStamped> samples{
    detected_pose(94), detected_pose(101), detected_pose(0), detected_pose(-1),
    detected_pose(99), detected_pose(96)};
  samples[4].header.stamp.nanosec = 1000000000U;
  TestContext context;
  std::size_t supplied = 0U;
  context.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation &&
        supplied < samples.size())
      {
        buffer.updateCubePose(samples[supplied++]);
      }
    };
  context.checkpoint_hook = [&](const std::size_t count) {
      if (count > 20U) {
        context.stopped = true;
      }
    };
  const auto result = provider.observe(instance(), profile(), context);
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 96);
  EXPECT_EQ(supplied, samples.size());
}

TEST(V2DetectedObservationProvider, ReplayedPoseWaitsForStrictlyNewerStamp)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  int now = 100;
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    [&]() {return detected_pose(now).header.stamp;});
  TestContext first;
  first.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        buffer.updateCubePose(detected_pose(100));
      }
    };
  ASSERT_TRUE(provider.observe(instance(), profile(), first).succeeded());
  TestContext second;
  std::size_t supplied = 0U;
  second.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        ++supplied;
        now = supplied == 1U ? 100 : 101;
        buffer.updateCubePose(detected_pose(now));
      }
    };
  second.checkpoint_hook = [&](const std::size_t count) {
      if (count > 10U) {
        second.stopped = true;
      }
    };
  const auto result = provider.observe(instance(), profile(), second);
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 101);
  EXPECT_GE(supplied, 2U);
}

TEST(V2DetectedObservationProvider, ZeroClockWaitsAndRollbackAllowsNewEpochObservation)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  int now = 100;
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    [&]() {return detected_pose(now).header.stamp;});
  TestContext first;
  first.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        buffer.updateCubePose(detected_pose(100));
      }
    };
  ASSERT_TRUE(provider.observe(instance(), profile(), first).succeeded());
  now = 0;
  TestContext reset;
  std::size_t supplied = 0U;
  reset.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        ++supplied;
        if (supplied == 1U) {
          buffer.updateCubePose(detected_pose(100));
        } else {
          now = 3;
          buffer.updateCubePose(detected_pose(2));
        }
      }
    };
  reset.checkpoint_hook = [&](const std::size_t count) {
      if (count > 10U) {
        reset.stopped = true;
      }
    };
  const auto result = provider.observe(instance(), profile(), reset);
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 2);
  EXPECT_GE(supplied, 2U);
}

TEST(V2DetectedObservationProvider, SampleExpiringDuringExactTfWaitRequiresNewObservation)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  int now = 100;
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, {"cube", "panda_link0"},
    [&]() {return detected_pose(now).header.stamp;});
  TestContext context;
  std::size_t supplied = 0U;
  context.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        buffer.updateCubePose(detected_pose(++supplied == 1U ? 99 : 106, "camera"));
      }
    };
  context.wait_hook = [&](const std::size_t) {
      now = 106;
      EXPECT_TRUE(tf_buffer->setTransform(
          exact_transform(99, "panda_link0", "camera"), "test", false));
      EXPECT_TRUE(tf_buffer->setTransform(
          exact_transform(106, "panda_link0", "camera"), "test", false));
    };
  context.checkpoint_hook = [&](const std::size_t count) {
      if (count > 10U) {
        context.stopped = true;
      }
    };
  const auto result = provider.observe(instance(), profile(), context);
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 106);
  EXPECT_EQ(supplied, 2U);
}

TEST(V2DetectedObservationProvider, ConfiguredAgeBudgetAllowsBoundedInferenceLatency)
{
  franka_motion::DetectedPoseBuffer buffer;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  franka_motion::workflow_v2::DetectedObservationProviderConfig config;
  config.object_id = "cube";
  config.target_frame = "panda_link0";
  config.max_observation_age_sec = 12.5;
  franka_motion::workflow_v2::DetectedObservationProvider provider(
    buffer, tf_buffer, config, []() {return detected_pose(100).header.stamp;});
  TestContext context;
  context.waiting_hook = [&](const auto reason) {
      if (reason == manipulation::workflow::TransferWaitingReason::kObservation) {
        buffer.updateCubePose(detected_pose(90));
      }
    };
  context.checkpoint_hook = [&](const std::size_t count) {
      if (count > 5U) {
        context.stopped = true;
      }
    };
  const auto result = provider.observe(instance(), profile(), context);
  ASSERT_TRUE(result.succeeded());
  EXPECT_EQ(result.value()->pose().header.stamp.sec, 90);
}
