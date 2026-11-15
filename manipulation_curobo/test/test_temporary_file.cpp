// Responsibility: Verifies exclusive temporary-file ownership, move semantics, and cleanup.

#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <utility>

#include "manipulation_curobo/temporary_file.hpp"

namespace
{

using manipulation_curobo::curobo::TemporaryFile;

TEST(TemporaryFile, OwnsAndRemovesCreatedPath)
{
  std::filesystem::path owned_path;
  {
    TemporaryFile file{"franka_curobo_test"};
    owned_path = file.path();
    EXPECT_TRUE(std::filesystem::is_regular_file(owned_path));
  }
  EXPECT_FALSE(std::filesystem::exists(owned_path));
}

TEST(TemporaryFile, MoveTransfersExactlyOneCleanupOwner)
{
  std::filesystem::path owned_path;
  {
    TemporaryFile source{"franka_curobo_move"};
    owned_path = source.path();
    TemporaryFile destination{std::move(source)};
    EXPECT_TRUE(source.path().empty());
    EXPECT_EQ(destination.path(), owned_path);
  }
  EXPECT_FALSE(std::filesystem::exists(owned_path));
}

TEST(TemporaryFile, RejectsEmptyOrPathLikePrefixes)
{
  EXPECT_THROW(TemporaryFile{""}, std::invalid_argument);
  EXPECT_THROW(TemporaryFile{"../escape"}, std::invalid_argument);
  EXPECT_THROW(TemporaryFile{"has space"}, std::invalid_argument);
}

}  // namespace
