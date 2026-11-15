// Responsibility: Check quiescence evidence without ROS callbacks, transport, or wall-clock waits.

#include <chrono>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "quiescence_policy.hpp"

namespace manipulation::moveit::detail
{
namespace
{
using namespace std::chrono_literals;
constexpr std::int64_t kEpoch = 1000000000LL;
const JointTarget kTarget{{"joint_a", "joint_b"}, {0.5, 1.0}, {}};

RequiredJointSample sample(std::int64_t offset, bool velocity = true)
{
  return {{0.5, 1.0}, velocity ? std::optional<std::vector<double>>{{0.0, 0.0}} :
    std::nullopt, kEpoch + offset};
}

QuiescenceEvidence observe(
  QuiescenceTracker & tracker, RequiredJointSample value,
  std::chrono::milliseconds received = 0ms,
  std::int64_t now = kEpoch + 200000000LL)
{
  std::string error;
  return tracker.observe(
    value, kTarget, kEpoch, now, std::chrono::steady_clock::time_point{} + received, error);
}

TEST(QuiescencePolicy, RequiresTwoDistinctLowVelocitySamplesAtTarget)
{
  QuiescenceTracker tracker;
  EXPECT_EQ(observe(tracker, sample(1)), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(2)), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, MovingOrOutsideTargetSamplesBreakConsecutiveEvidence)
{
  QuiescenceTracker tracker;
  EXPECT_EQ(observe(tracker, sample(1)), QuiescenceEvidence::kPending);
  auto moving = sample(2);
  moving.velocities = {{0.0, 0.02}};
  EXPECT_EQ(observe(tracker, moving), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(3)), QuiescenceEvidence::kPending);
  auto displaced = sample(4);
  displaced.positions[0] += 0.02;
  EXPECT_EQ(observe(tracker, displaced), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(5)), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(6)), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, PositionOnlyEvidenceRequiresFiveSamplesAndFullSteadyWindow)
{
  QuiescenceTracker tracker;
  for (int index = 0; index < 5; ++index) {
    EXPECT_EQ(observe(tracker, sample(index + 1, false), index * 20ms),
      QuiescenceEvidence::kPending);
  }
  EXPECT_EQ(observe(tracker, sample(6, false), 100ms), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, PositionExcursionRestartsWindow)
{
  QuiescenceTracker tracker;
  EXPECT_EQ(observe(tracker, sample(1, false), 0ms), QuiescenceEvidence::kPending);
  auto displaced = sample(2, false);
  displaced.positions[0] += 0.002;
  EXPECT_EQ(observe(tracker, displaced, 100ms), QuiescenceEvidence::kPending);
  for (int index = 0; index < 4; ++index) {
    displaced.stamp_nanoseconds++;
    EXPECT_EQ(observe(tracker, displaced, 110ms + index * 20ms),
      QuiescenceEvidence::kPending);
  }
  displaced.stamp_nanoseconds++;
  EXPECT_EQ(observe(tracker, displaced, 200ms), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, StaleEvidenceWaitsForFreshSamples)
{
  QuiescenceTracker tracker;
  EXPECT_EQ(observe(tracker, sample(-1)), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(1), 0ms, kEpoch + 300000000LL),
    QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(100000000LL)), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(100000001LL)), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, DecreasingReplayCannotLowerHighWaterMark)
{
  QuiescenceTracker tracker;
  EXPECT_EQ(observe(tracker, sample(10)), QuiescenceEvidence::kPending);
  for (const auto stamp : {8, 9, 10}) {
    EXPECT_EQ(observe(tracker, sample(stamp)), QuiescenceEvidence::kPending);
  }
  EXPECT_EQ(observe(tracker, sample(11)), QuiescenceEvidence::kPending);
  EXPECT_EQ(observe(tracker, sample(12)), QuiescenceEvidence::kReady);
}

TEST(QuiescencePolicy, InvalidNumericAndTemporalEvidenceFailsClosed)
{
  QuiescenceTracker tracker;
  auto malformed = sample(1);
  malformed.stamp_nanoseconds = 0;
  EXPECT_EQ(observe(tracker, malformed), QuiescenceEvidence::kInvalid);
  malformed = sample(1);
  malformed.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(observe(tracker, malformed), QuiescenceEvidence::kInvalid);
  EXPECT_EQ(observe(tracker, sample(200000001LL)), QuiescenceEvidence::kInvalid);
  EXPECT_NE(tracker.diagnostic(kTarget).find("epoch_barrier_ns=1000000000"),
    std::string::npos);
}

TEST(QuiescencePolicy, ClockRollbackAndZeroRejectTheCurrentEpoch)
{
  QuiescenceClock clock{kEpoch};
  std::string error;
  EXPECT_TRUE(clock.observe(kEpoch, error));
  EXPECT_TRUE(clock.observe(kEpoch + 10, error));
  EXPECT_FALSE(clock.observe(kEpoch + 9, error));
  EXPECT_NE(error.find("previous_now_ns=1000000010"), std::string::npos);
  EXPECT_NE(error.find("epoch_barrier_ns=1000000000"), std::string::npos);
  EXPECT_FALSE(clock.observe(0, error));
}

}  // namespace
}  // namespace manipulation::moveit::detail
