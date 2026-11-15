// Responsibility: Resolve optional package:// planner resources without
// coupling the generic backend to a robot package.

#ifndef PACKAGE_RESOURCE_HPP_
#define PACKAGE_RESOURCE_HPP_

#include <string>

namespace manipulation_curobo::detail
{

/// Return a filesystem path for package:// URIs and preserve ordinary paths.
/**
 * @throws std::invalid_argument for malformed or parent-traversing package
 * URIs, and ament_index_cpp::PackageNotFoundError for unknown packages.
 */
std::string resolvePackageResource(const std::string & resource);

}  // namespace manipulation_curobo::detail

#endif  // PACKAGE_RESOURCE_HPP_
