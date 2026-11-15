// Responsibility: Declares the private YAML boundary for the V2 instance/profile catalog.

#ifndef WORKFLOW_V2__OBJECT_CATALOG_YAML_HPP_
#define WORKFLOW_V2__OBJECT_CATALOG_YAML_HPP_

#include <filesystem>
#include <string_view>

#include "manipulation/workflow/object_catalog.hpp"

namespace franka_motion::workflow_v2
{

/** Parse the versioned catalog schema without performing resource I/O. */
[[nodiscard]] manipulation::workflow::ObjectCatalog parse_object_catalog_yaml(
  std::string_view yaml);

/** Load and parse one catalog file. Package URIs are resolved by the composition root. */
[[nodiscard]] manipulation::workflow::ObjectCatalog load_object_catalog_yaml(
  const std::filesystem::path & path);

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__OBJECT_CATALOG_YAML_HPP_
