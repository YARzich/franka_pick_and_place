// Responsibility: Verify target geometry policy without workflow, lease, or ROS runtime fixtures.

#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "gtest/gtest.h"

#include "detail/transfer_target_resolution.hpp"
#include "transfer_domain_test_support.hpp"

namespace manipulation::workflow
{
namespace
{
using Step = manipulation_msgs::msg::TransferObjectStepV2;

TEST(TransferTargetResolution, AbsolutePoseIsPreservedWithoutMutatingInputs)
{
  Step step;
  step.target_mode = Step::TARGET_ABSOLUTE_POSE;
  step.target_object_pose = test_support::stamped_pose("world", 2.0);
  const auto original = step;
  const auto observed = test_support::observation();
  EXPECT_EQ(detail::resolve_transfer_target(step, observed, test_support::profile(), {}),
    original.target_object_pose);
  EXPECT_EQ(step, original);
}

TEST(TransferTargetResolution, TranslationUsesCurrentSourceAndKeepsStampAndOrientation)
{
  Step step;
  step.target_mode = Step::TARGET_TRANSLATION_FROM_CURRENT_SOURCE;
  step.target_translation.vector.x = 1.0;
  step.target_translation.vector.y = -2.0;
  step.target_translation.vector.z = 3.0;
  const auto observed = test_support::observation();
  const auto original = observed.pose();
  const auto target = detail::resolve_transfer_target(step, observed, test_support::profile(), {});
  EXPECT_EQ(target.header, original.header);
  EXPECT_EQ(target.pose.orientation, original.pose.orientation);
  EXPECT_DOUBLE_EQ(target.pose.position.x, original.pose.position.x + 1.0);
  EXPECT_DOUBLE_EQ(target.pose.position.y, original.pose.position.y - 2.0);
  EXPECT_DOUBLE_EQ(target.pose.position.z, original.pose.position.z + 3.0);
  EXPECT_EQ(observed.pose(), original);
}

TEST(TransferTargetResolution, InitialSourceWithoutSymmetryKeepsOriginalOrientation)
{
  Step step;
  step.object_id = "cube_1";
  step.target_mode = Step::TARGET_INITIAL_SOURCE;
  auto initial = test_support::stamped_pose("world", 4.0);
  initial.pose.orientation.w = -1.0;
  const std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> sources{
    {step.object_id, initial}};
  EXPECT_EQ(detail::resolve_transfer_target(
      step, test_support::observation(), test_support::profile(), sources), initial);
  EXPECT_EQ(sources.at(step.object_id), initial);
}

TEST(TransferTargetResolution, ChoosesNearestDeclaredLocalSymmetryAndQuaternionSign)
{
  Step step;
  step.object_id = "cube_1";
  step.target_mode = Step::TARGET_INITIAL_SOURCE;
  const auto initial = test_support::stamped_pose("world", 4.0);
  geometry_msgs::msg::Quaternion symmetry;
  symmetry.z = std::sqrt(0.5);
  symmetry.w = std::sqrt(0.5);
  auto current = test_support::stamped_pose();
  current.pose.orientation.z = -symmetry.z;
  current.pose.orientation.w = -symmetry.w;
  const auto observed = ObjectObservation::create(step.object_id, current, "new");
  const auto base = test_support::profile();
  const auto profile = ObjectProfile::create(
    base.profile_id(), base.geometry(), std::nullopt, {initial.pose.orientation, symmetry});
  const auto target = detail::resolve_transfer_target(
    step, observed, profile, {{step.object_id, initial}});
  EXPECT_EQ(target.header, initial.header);
  EXPECT_EQ(target.pose.position, initial.pose.position);
  EXPECT_DOUBLE_EQ(target.pose.orientation.z, current.pose.orientation.z);
  EXPECT_DOUBLE_EQ(target.pose.orientation.w, current.pose.orientation.w);
}

TEST(TransferTargetResolution, MissingInitialSourceAndUnknownModeFailExplicitly)
{
  Step step;
  step.object_id = "cube_1";
  step.target_mode = Step::TARGET_INITIAL_SOURCE;
  EXPECT_THROW(detail::resolve_transfer_target(
      step, test_support::observation(), test_support::profile(), {}), std::invalid_argument);
  step.target_mode = 255U;
  EXPECT_THROW(detail::resolve_transfer_target(
      step, test_support::observation(), test_support::profile(), {}), std::invalid_argument);
}
}  // namespace
}  // namespace manipulation::workflow
