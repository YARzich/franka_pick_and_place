// Responsibility: Verifies service-request correlation, timeout, and late-response handling.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "service_request_tracker.hpp"

namespace manipulation::monitoring::detail
{
namespace
{

using namespace std::chrono_literals;

TEST(ServiceRequestTracker, AllowsOnlyOneRequestAndAcceptsItsResponse)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{10s};

  const auto token = tracker.tryStart(start, 2s);
  ASSERT_TRUE(token);
  EXPECT_FALSE(tracker.tryStart(start + 1ms, 2s));
  EXPECT_TRUE(tracker.bindRequestId(*token, 17));

  const auto pending = tracker.snapshot(start + 500ms);
  EXPECT_TRUE(pending.in_flight);
  EXPECT_EQ(pending.generation, token->generation);
  ASSERT_TRUE(pending.request_id);
  EXPECT_EQ(*pending.request_id, 17);
  EXPECT_EQ(pending.age, 500ms);

  EXPECT_EQ(
    tracker.acceptResponse(*token, start + 1s),
    ServiceResponseDisposition::kAccept);
  EXPECT_FALSE(tracker.snapshot(start + 1s).in_flight);
}

TEST(ServiceRequestTracker, ExpiryInvalidatesLateCallbackWithoutClearingNewRequest)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{20s};
  const auto first = tracker.tryStart(start, 1s);
  ASSERT_TRUE(first);
  ASSERT_TRUE(tracker.bindRequestId(*first, 41));

  const auto expired = tracker.expire(start + 1s);
  ASSERT_TRUE(expired);
  EXPECT_EQ(expired->token.generation, first->generation);
  ASSERT_TRUE(expired->request_id);
  EXPECT_EQ(*expired->request_id, 41);
  EXPECT_EQ(expired->age, 1s);

  const auto second = tracker.tryStart(start + 1100ms, 1s);
  ASSERT_TRUE(second);
  ASSERT_TRUE(tracker.bindRequestId(*second, 42));
  EXPECT_EQ(
    tracker.acceptResponse(*first, start + 1200ms),
    ServiceResponseDisposition::kStaleGeneration);

  const auto current = tracker.snapshot(start + 1200ms);
  EXPECT_TRUE(current.in_flight);
  EXPECT_EQ(current.generation, second->generation);
  ASSERT_TRUE(current.request_id);
  EXPECT_EQ(*current.request_id, 42);
  EXPECT_EQ(current.timeout_count, 1U);
}

TEST(ServiceRequestTracker, RejectsResponseThatArrivesPastDeadline)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{30s};
  const auto token = tracker.tryStart(start, 2s);
  ASSERT_TRUE(token);

  EXPECT_EQ(
    tracker.acceptResponse(*token, start + 2s),
    ServiceResponseDisposition::kDeadlineExceeded);
  const auto snapshot = tracker.snapshot(start + 2s);
  EXPECT_FALSE(snapshot.in_flight);
  EXPECT_EQ(snapshot.timeout_count, 1U);
}

TEST(ServiceRequestTracker, LateRequestIdBindingRequestsMiddlewareCleanup)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{40s};
  const auto token = tracker.tryStart(start, 1s);
  ASSERT_TRUE(token);
  ASSERT_TRUE(tracker.expire(start + 1s));

  EXPECT_FALSE(tracker.bindRequestId(*token, 99));
}

TEST(ServiceRequestTracker, AbandonAndInvalidTimeoutPreserveReusableSlot)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{50s};

  EXPECT_THROW(tracker.tryStart(start, 0ns), std::invalid_argument);
  EXPECT_THROW(tracker.tryStart(start, -1ns), std::invalid_argument);

  const auto token = tracker.tryStart(start, 1s);
  ASSERT_TRUE(token);
  EXPECT_TRUE(tracker.abandon(*token));
  EXPECT_FALSE(tracker.abandon(*token));
  EXPECT_EQ(
    tracker.acceptResponse(*token, start + 1ms),
    ServiceResponseDisposition::kStaleGeneration);
  EXPECT_TRUE(tracker.tryStart(start + 1ms, 1s));
}

TEST(ServiceRequestTracker, ClampsInjectedBackwardTimeToZeroAge)
{
  ServiceRequestTracker tracker;
  const ServiceRequestTracker::TimePoint start{60s};
  ASSERT_TRUE(tracker.tryStart(start, 1s));

  EXPECT_EQ(tracker.snapshot(ServiceRequestTracker::TimePoint{10s}).age, 0ns);
}

}  // namespace
}  // namespace manipulation::monitoring::detail
