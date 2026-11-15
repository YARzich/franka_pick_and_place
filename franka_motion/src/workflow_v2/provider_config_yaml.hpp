// Responsibility: Declares versioned YAML loaders for V2 observation and grasp providers.

#ifndef WORKFLOW_V2__PROVIDER_CONFIG_YAML_HPP_
#define WORKFLOW_V2__PROVIDER_CONFIG_YAML_HPP_

#include <filesystem>
#include <string_view>

#include "workflow_v2/mtc_grasp_provider.hpp"
#include "workflow_v2/observation_providers.hpp"

namespace franka_motion::workflow_v2
{

[[nodiscard]] ConfiguredObservationPreset parse_configured_observation_yaml(
  std::string_view yaml);
[[nodiscard]] ConfiguredObservationPreset load_configured_observation_yaml(
  const std::filesystem::path & path);

[[nodiscard]] DetectedObservationProviderConfig parse_detected_observation_yaml(
  std::string_view yaml);
[[nodiscard]] DetectedObservationProviderConfig load_detected_observation_yaml(
  const std::filesystem::path & path);

[[nodiscard]] MtcGraspProfiles parse_mtc_grasp_profiles_yaml(std::string_view yaml);
[[nodiscard]] MtcGraspProfiles load_mtc_grasp_profiles_yaml(
  const std::filesystem::path & path);

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__PROVIDER_CONFIG_YAML_HPP_
