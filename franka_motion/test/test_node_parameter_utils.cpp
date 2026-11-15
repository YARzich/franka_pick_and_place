// Responsibility: Verifies parameter declaration preserves overrides and supplies defaults.

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>

#include "franka_motion/node_parameter_utils.hpp"

namespace
{
class NodeParameterUtilsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
}  // namespace

TEST_F(NodeParameterUtilsTest, DeclaresMissingParameterWithDefault)
{
  const auto node = rclcpp::Node::make_shared("test_node_parameter_default");

  const auto value = franka_motion::declareOrGetParameter<std::string>(
    node, "frame_id", "world");

  EXPECT_EQ(value, "world");
  EXPECT_TRUE(node->has_parameter("frame_id"));
}

TEST_F(NodeParameterUtilsTest, PreservesAutomaticallyDeclaredOverride)
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("count", std::int64_t{7})});
  const auto node = std::make_shared<rclcpp::Node>("test_node_parameter_override", options);

  const auto value = franka_motion::declareOrGetParameter<std::int64_t>(
    node, "count", 3);

  EXPECT_EQ(value, 7);
}
