// Responsibility: Verifies bounded result-delivery grace after one-shot child-process termination.

#include <gtest/gtest.h>

#include <chrono>

#include "one_shot_process_result_wait_policy.hpp"

namespace
{
using namespace std::chrono_literals;
using franka_motion::OneShotProcessResultWaitPolicy;
using franka_motion::OneShotProcessState;
}

TEST(OneShotProcessResultWaitPolicy, StopsImmediatelyAfterFailedExit)
{
  OneShotProcessResultWaitPolicy policy{1s};
  const auto now = OneShotProcessResultWaitPolicy::Clock::time_point{};

  EXPECT_TRUE(policy.shouldKeepWaiting(OneShotProcessState::kRunning, now));
  EXPECT_FALSE(policy.shouldKeepWaiting(OneShotProcessState::kFailed, now + 1ms));
}

TEST(OneShotProcessResultWaitPolicy, AllowsBoundedDeliveryGraceAfterSuccessfulExit)
{
  OneShotProcessResultWaitPolicy policy{1s};
  const auto exit_observed_at = OneShotProcessResultWaitPolicy::Clock::time_point{};

  EXPECT_TRUE(
    policy.shouldKeepWaiting(OneShotProcessState::kSucceeded, exit_observed_at));
  EXPECT_TRUE(
    policy.shouldKeepWaiting(OneShotProcessState::kSucceeded, exit_observed_at + 999ms));
  EXPECT_FALSE(
    policy.shouldKeepWaiting(OneShotProcessState::kSucceeded, exit_observed_at + 1s));
}

TEST(OneShotProcessResultWaitPolicy, ClampsNegativeDeliveryGraceToZero)
{
  OneShotProcessResultWaitPolicy policy{-1ms};

  EXPECT_FALSE(policy.shouldKeepWaiting(
      OneShotProcessState::kSucceeded,
      OneShotProcessResultWaitPolicy::Clock::time_point{}));
}
