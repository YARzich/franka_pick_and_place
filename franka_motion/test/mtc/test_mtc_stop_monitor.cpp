// Responsibility: Verifies bounded stop forwarding and polling-thread cleanup.

#include <atomic>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "mtc_stop_monitor.hpp"

namespace
{

using namespace std::chrono_literals;

TEST(MtcStopMonitor, ForwardsOneStopAndJoins)
{
  std::atomic<bool> stop_requested{false};
  std::atomic<int> stop_calls{0};
  franka_motion::detail::MtcStopMonitor monitor(
    [&stop_requested]() {return stop_requested.load();},
    [&stop_calls]() {stop_calls.fetch_add(1);});

  stop_requested.store(true);
  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (stop_calls.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  monitor.finish();

  EXPECT_EQ(stop_calls.load(), 1);
}

TEST(MtcStopMonitor, FinishWithoutStopDoesNotInvokeCallback)
{
  std::atomic<int> stop_calls{0};
  franka_motion::detail::MtcStopMonitor monitor(
    []() {return false;},
    [&stop_calls]() {stop_calls.fetch_add(1);});

  monitor.finish();

  EXPECT_EQ(stop_calls.load(), 0);
}

TEST(MtcStopMonitor, BoundedStopCombinesCancellationAndDeadline)
{
  bool stop_requested = false;
  auto stop = franka_motion::detail::make_bounded_stop_request(
    [&stop_requested]() {return stop_requested;},
    std::chrono::steady_clock::now() + 1s);

  EXPECT_FALSE(stop());
  stop_requested = true;
  EXPECT_TRUE(stop());

  auto expired = franka_motion::detail::make_bounded_stop_request(
    {}, std::chrono::steady_clock::now() - 1ms);
  EXPECT_TRUE(expired());
}

TEST(MtcStopMonitor, LocalDeadlineNeverExtendsOuterRequest)
{
  const auto outer = std::chrono::steady_clock::now() + 20ms;

  EXPECT_EQ(franka_motion::detail::bounded_deadline(outer, 1s), outer);
  const auto before = std::chrono::steady_clock::now();
  const auto immediate = franka_motion::detail::bounded_deadline(
    std::chrono::steady_clock::time_point::max(), 0ms);
  const auto after = std::chrono::steady_clock::now();
  EXPECT_GE(immediate, before);
  EXPECT_LE(immediate, after);
}

}  // namespace
