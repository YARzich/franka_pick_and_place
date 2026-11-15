// Responsibility: Verifies the shared canonical syntax boundary for ROS frame IDs.

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <string_view>

#include "manipulation/validation/frame_id.hpp"

namespace
{

TEST(FrameIdValidation, AcceptsCanonicalRelativeFrames)
{
  constexpr std::array<std::string_view, 4> frames{
    "world",
    "base_link",
    "robot/camera_optical_frame",
    "cell_1/robot-2/tool.frame",
  };

  for (const auto frame : frames) {
    EXPECT_TRUE(
      manipulation::validation::is_canonical_relative_frame_id(frame)) << frame;
  }
}

TEST(FrameIdValidation, RejectsAliasesWhitespaceAndControlCharacters)
{
  constexpr std::array<std::string_view, 7> frames{
    "",
    "/world",
    "world/",
    "robot//camera",
    "camera link",
    "camera\tlink",
    "camera\nlink",
  };

  for (const auto frame : frames) {
    EXPECT_FALSE(
      manipulation::validation::is_canonical_relative_frame_id(frame)) << frame;
  }

  std::string embedded_control{"camera"};
  embedded_control.push_back('\0');
  embedded_control += "link";
  EXPECT_FALSE(
    manipulation::validation::is_canonical_relative_frame_id(embedded_control));

  std::string delete_control{"camera"};
  delete_control.push_back(static_cast<char>(0x7f));
  delete_control += "link";
  EXPECT_FALSE(
    manipulation::validation::is_canonical_relative_frame_id(delete_control));
}

TEST(FrameIdValidation, PreservesTheOriginalPublicAlias)
{
  const std::array<std::string, 3> frames{
    "world", "robot/camera", "robot//camera"};
  for (const auto & frame : frames) {
    EXPECT_EQ(
      manipulation::validation::is_relative_rep105_frame_id(frame),
      manipulation::validation::is_canonical_relative_frame_id(frame));
  }
}

}  // namespace
