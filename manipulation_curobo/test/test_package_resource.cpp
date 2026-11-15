// Responsibility: Verify package URI resolution stays deterministic and
// cannot escape the selected package share.

#include <filesystem>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "package_resource.hpp"

TEST(PackageResource, PreservesOrdinaryPaths)
{
  EXPECT_EQ(
    manipulation_curobo::detail::resolvePackageResource("/tmp/robot.yaml"),
    "/tmp/robot.yaml");
  EXPECT_EQ(
    manipulation_curobo::detail::resolvePackageResource("relative/robot.yaml"),
    "relative/robot.yaml");
}

TEST(PackageResource, ResolvesInstalledPackageShare)
{
  const std::filesystem::path expected =
    std::filesystem::path(
    ament_index_cpp::get_package_share_directory("manipulation_curobo")) / "README.md";

  EXPECT_EQ(
    manipulation_curobo::detail::resolvePackageResource(
      "package://manipulation_curobo/README.md"),
    expected.string());
}

TEST(PackageResource, RejectsMalformedOrEscapingUris)
{
  EXPECT_THROW(
    manipulation_curobo::detail::resolvePackageResource("package://missing_path"),
    std::invalid_argument);
  EXPECT_THROW(
    manipulation_curobo::detail::resolvePackageResource(
      "package://manipulation_curobo/../secret.yaml"),
    std::invalid_argument);
}
