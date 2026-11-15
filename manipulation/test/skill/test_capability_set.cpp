// Responsibility: Verifies canonical logical IDs and deterministic capability set operations.

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/logical_id.hpp"

namespace manipulation::skill
{
namespace
{

TEST(LogicalId, AcceptsCanonicalLowercaseSegments)
{
  for (const std::string value : {
        "arm", "arm/main", "motion/servo_pose", "planner/ompl.v2",
        "tool/gripper-1"})
  {
    EXPECT_TRUE(is_canonical_logical_id(value)) << value;
  }
}

TEST(LogicalId, RejectsNoncanonicalValues)
{
  for (const std::string value : {
        "", "/arm", "arm/", "arm//main", "Arm/main", "arm/1main",
        "arm main", "arm:main", "arm/m\xC3\xA4in"})
  {
    EXPECT_FALSE(is_canonical_logical_id(value)) << value;
  }
}

TEST(CapabilitySet, SortsAndDeduplicatesCapabilities)
{
  const CapabilitySet capabilities{
    "motion/servo_pose", "gripper/width", "motion/servo_pose",
    "motion/plan_execute"};

  const std::vector<std::string> expected{
    "gripper/width", "motion/plan_execute", "motion/servo_pose"};
  EXPECT_EQ(capabilities.values(), expected);
  EXPECT_EQ(capabilities.size(), expected.size());
  EXPECT_TRUE(capabilities.contains("motion/servo_pose"));
  EXPECT_FALSE(capabilities.contains("motion/force"));
  EXPECT_FALSE(capabilities.contains("Motion/servo_pose"));
}

TEST(CapabilitySet, RejectsMalformedCapability)
{
  EXPECT_THROW(
    CapabilitySet({"motion/plan_execute", "Motion/servo_pose"}),
    std::invalid_argument);
}

TEST(CapabilitySet, ReportsMissingRequirementsInSortedOrder)
{
  const CapabilitySet available{
    "gripper/width", "motion/plan_execute", "tf/exact_stamp"};
  const CapabilitySet supported{
    "motion/plan_execute", "gripper/width"};
  const CapabilitySet required{
    "world/rigid_object", "motion/plan_execute", "motion/servo_pose"};

  EXPECT_TRUE(available.satisfies(supported));
  EXPECT_FALSE(available.satisfies(required));
  EXPECT_EQ(
    available.missing(required),
    (std::vector<std::string>{"motion/servo_pose", "world/rigid_object"}));
  EXPECT_TRUE(available.missing(CapabilitySet{}).empty());
}

}  // namespace
}  // namespace manipulation::skill
