// Responsibility: Implement bounded package URI parsing for planner configs.

#include "package_resource.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace manipulation_curobo::detail
{
namespace
{

constexpr char kPackageScheme[] = "package://";

bool containsParentTraversal(const std::filesystem::path & path)
{
  for (const auto & component : path) {
    if (component == "..") {
      return true;
    }
  }
  return false;
}

}  // namespace

std::string resolvePackageResource(const std::string & resource)
{
  if (resource.rfind(kPackageScheme, 0U) != 0U) {
    return resource;
  }

  const std::string remainder = resource.substr(std::char_traits<char>::length(kPackageScheme));
  const auto separator = remainder.find('/');
  if (separator == std::string::npos || separator == 0U || separator + 1U >= remainder.size()) {
    throw std::invalid_argument("package URI must contain a package and relative path");
  }

  const std::string package_name = remainder.substr(0U, separator);
  const std::filesystem::path relative_path = remainder.substr(separator + 1U);
  if (relative_path.is_absolute() || containsParentTraversal(relative_path)) {
    throw std::invalid_argument("package URI path must stay inside its package share");
  }

  const std::filesystem::path package_share =
    ament_index_cpp::get_package_share_directory(package_name);
  return (package_share / relative_path).lexically_normal().string();
}

}  // namespace manipulation_curobo::detail
