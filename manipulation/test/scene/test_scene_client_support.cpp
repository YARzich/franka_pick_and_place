// Responsibility: Verifies scene-client response validation, revision handling, and lease metadata.

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "manipulation/scene/scene_client_support.hpp"

namespace
{

using manipulation::scene::LeaseReleaseGuard;

TEST(SceneClientSupport, ConvertsPositiveSecondsToRosDuration)
{
  const auto duration =
    manipulation::scene::duration_from_positive_seconds(1.25);
  EXPECT_EQ(duration.sec, 1);
  EXPECT_EQ(duration.nanosec, 250000000U);

  const auto rounded =
    manipulation::scene::duration_from_positive_seconds(0.9999999996);
  EXPECT_EQ(rounded.sec, 1);
  EXPECT_EQ(rounded.nanosec, 0U);
}

TEST(SceneClientSupport, RejectsInvalidOrUnrepresentableDuration)
{
  EXPECT_THROW(
    (void)manipulation::scene::duration_from_positive_seconds(0.0),
    std::invalid_argument);
  EXPECT_THROW(
    (void)manipulation::scene::duration_from_positive_seconds(
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  EXPECT_THROW(
    (void)manipulation::scene::duration_from_positive_seconds(
      static_cast<double>(std::numeric_limits<std::int32_t>::max()) + 1.0),
    std::invalid_argument);
}

TEST(SceneClientSupport, BuildsDeterministicReadableOperationId)
{
  const auto operation_id = manipulation::scene::make_scene_operation_id(
    "owner", "add", "table", 0x00000001U, 0xabcdef01U);
  EXPECT_EQ(operation_id, "owner/add/table/00000001abcdef01");
  EXPECT_NE(
    operation_id,
    manipulation::scene::make_scene_operation_id(
      "owner", "add", "table", 0x00000001U, 0xabcdef02U));
  EXPECT_THROW(
    manipulation::scene::make_scene_operation_id(
      "", "add", "table", 1U, 2U),
    std::invalid_argument);
}

TEST(SceneClientSupport, ExplicitReleaseRunsExactlyOnce)
{
  int calls = 0;
  {
    LeaseReleaseGuard guard([&calls]() {
        ++calls;
        return true;
      });
    EXPECT_TRUE(guard.active());
    EXPECT_TRUE(guard.release());
    EXPECT_FALSE(guard.active());
    EXPECT_TRUE(guard.release());
  }
  EXPECT_EQ(calls, 1);
}

TEST(SceneClientSupport, DestructorRetriesFailedExplicitRelease)
{
  int calls = 0;
  {
    LeaseReleaseGuard guard([&calls]() {
        ++calls;
        return calls > 1;
      });
    EXPECT_FALSE(guard.release());
    EXPECT_TRUE(guard.active());
  }
  EXPECT_EQ(calls, 2);
}

TEST(SceneClientSupport, CallbackExceptionBecomesReleaseFailure)
{
  int calls = 0;
  {
    LeaseReleaseGuard guard([&calls]() -> bool {
        ++calls;
        throw std::runtime_error("release failed");
      });
    EXPECT_FALSE(guard.release());
  }
  EXPECT_EQ(calls, 2);
}

}  // namespace
