// Responsibility: Verify catalog/provider schemas and the shipped cube presets.

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "workflow_v2/object_catalog_yaml.hpp"
#include "workflow_v2/provider_config_yaml.hpp"

namespace
{

constexpr const char * kCatalogFixture =
  R"yaml(
schema_version: 2
profiles:
  crate:
    grasp_profile_id: crate_grasp
    orientation_symmetries_xyzw:
      - [0.0, 0.0, 0.0, 1.0]
      - [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]
    geometry:
      kind: primitive
      primitive_type: box
      dimensions_m: [0.1, 0.2, 0.3]
  part:
    geometry:
      kind: mesh
      resource_uri: package://demo/part.stl
      scale_xyz: [1.0, 2.0, 3.0]
  tool:
    geometry:
      kind: compound
      parts:
        - kind: primitive
          primitive_type: cylinder
          dimensions_m: [0.2, 0.03]
        - kind: mesh
          resource_uri: package://demo/handle.stl
          local_pose:
            position_xyz: [0.0, 0.0, 0.1]
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
objects:
  crate_a: {profile_id: crate}
  part_a: {profile_id: part}
  tool_a: {profile_id: tool}
)yaml";

std::filesystem::path source_config(const std::string & name)
{
  return std::filesystem::path{FRANKA_MOTION_SOURCE_DIR} / "config" / "tasks" / name;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::ostringstream output;
  output << input.rdbuf();
  return output.str();
}

}  // namespace

TEST(V2ObjectCatalogYaml, ParsesPrimitiveMeshCompoundAndDistinctInstances)
{
  const auto catalog = franka_motion::workflow_v2::parse_object_catalog_yaml(kCatalogFixture);

  EXPECT_EQ(catalog.profile_count(), 3U);
  EXPECT_EQ(catalog.instance_count(), 3U);
  EXPECT_EQ(
    catalog.profile_for("crate_a").geometry().kind(),
    manipulation::workflow::ObjectGeometryKind::kPrimitive);
  EXPECT_EQ(
    catalog.profile_for("part_a").geometry().kind(),
    manipulation::workflow::ObjectGeometryKind::kMesh);
  EXPECT_EQ(
    catalog.profile_for("tool_a").geometry().kind(),
    manipulation::workflow::ObjectGeometryKind::kCompound);
  EXPECT_EQ(catalog.profile_for("tool_a").geometry().parts().size(), 2U);
  ASSERT_EQ(catalog.profile_for("crate_a").orientation_symmetries().size(), 2U);
  EXPECT_DOUBLE_EQ(
    catalog.profile_for("crate_a").orientation_symmetries().back().z,
    0.7071067811865476);
}

TEST(V2ObjectCatalogYaml, RejectsMalformedOrInvalidOrientationSymmetries)
{
  constexpr const char * malformed =
    R"yaml(
schema_version: 2
profiles:
  cube:
    orientation_symmetries_xyzw: [0.0, 0.0, 0.0, 1.0]
    geometry: {kind: primitive, primitive_type: box, dimensions_m: [0.05, 0.05, 0.05]}
objects:
  cube: {profile_id: cube}
)yaml";
  constexpr const char * non_unit =
    R"yaml(
schema_version: 2
profiles:
  cube:
    orientation_symmetries_xyzw:
      - [0.0, 0.0, 0.0, 2.0]
    geometry: {kind: primitive, primitive_type: box, dimensions_m: [0.05, 0.05, 0.05]}
objects:
  cube: {profile_id: cube}
)yaml";

  EXPECT_THROW(
    static_cast<void>(franka_motion::workflow_v2::parse_object_catalog_yaml(malformed)),
    std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(franka_motion::workflow_v2::parse_object_catalog_yaml(non_unit)),
    std::invalid_argument);
}

TEST(V2ObjectCatalogYaml, LoadsShippedRuntimeCubeWithoutSourcePose)
{
  const auto path = source_config("object_catalog.yaml");
  const auto text = read_file(path);
  const auto catalog = franka_motion::workflow_v2::load_object_catalog_yaml(path);

  EXPECT_EQ(catalog.require_instance("cube").profile_id(), "cube_50mm");
  EXPECT_EQ(catalog.profile_for("cube").grasp_profile_id(), "panda_top_down_cube");
  EXPECT_EQ(catalog.profile_for("cube").orientation_symmetries().size(), 24U);
  EXPECT_EQ(text.find("source_pose"), std::string::npos);
  EXPECT_EQ(text.find("position_xyz"), std::string::npos);
}

TEST(V2ProviderYaml, KeepsWorldObservationAndGraspProfilesInSeparateFiles)
{
  const auto gazebo = franka_motion::workflow_v2::load_configured_observation_yaml(
    source_config("configured_observations_gazebo.yaml"));
  const auto isaac = franka_motion::workflow_v2::load_configured_observation_yaml(
    source_config("configured_observations_isaac.yaml"));
  const auto detected = franka_motion::workflow_v2::load_detected_observation_yaml(
    source_config("detected_observation.yaml"));
  const auto grasps = franka_motion::workflow_v2::load_mtc_grasp_profiles_yaml(
    source_config("mtc_grasp_profiles.yaml"));

  ASSERT_EQ(gazebo.size(), 1U);
  ASSERT_EQ(isaac.size(), 1U);
  EXPECT_DOUBLE_EQ(gazebo.at("cube").pose.position.x, 0.405);
  EXPECT_DOUBLE_EQ(isaac.at("cube").pose.position.x, 0.4);
  EXPECT_EQ(detected.object_id, "cube");
  EXPECT_EQ(detected.target_frame, "panda_link0");
  EXPECT_DOUBLE_EQ(detected.max_observation_age_sec, 5.0);
  // The shipped preset must keep the resting cube out of the tabletop
  // collision box exactly like the configured observations do.
  ASSERT_TRUE(detected.minimum_position_z.has_value());
  EXPECT_DOUBLE_EQ(*detected.minimum_position_z, 0.027);
  ASSERT_EQ(grasps.size(), 1U);
  const auto & grasp = grasps.at("panda_top_down_cube");
  EXPECT_EQ(grasp.arm_group_name, "panda_arm");
  EXPECT_DOUBLE_EQ(grasp.gripper_opening_m, 0.0);
  EXPECT_DOUBLE_EQ(grasp.gripper_force_n, 0.0);
}

TEST(V2ProviderYaml, DetectedSupportBoundIsOptionalButMustBeFinite)
{
  const auto minimal = franka_motion::workflow_v2::parse_detected_observation_yaml(
    R"yaml(
schema_version: 1
provider_id: detected
object_id: cube
target_frame: panda_link0
)yaml");
  EXPECT_FALSE(minimal.minimum_position_z.has_value());

  EXPECT_THROW(
    franka_motion::workflow_v2::parse_detected_observation_yaml(
      R"yaml(
schema_version: 1
provider_id: detected
object_id: cube
target_frame: panda_link0
minimum_position_z_m: .nan
)yaml"),
    std::invalid_argument);
}


TEST(V2ProviderYaml, ContainsNoSemanticWaitOrPollingConfiguration)
{
  const std::string names[] = {
    "object_catalog.yaml",
    "configured_observations_gazebo.yaml",
    "configured_observations_isaac.yaml",
    "detected_observation.yaml",
    "mtc_grasp_profiles.yaml",
  };
  const std::string forbidden[] = {"timeout", "poll", "retry", "heartbeat"};
  for (const auto & name : names) {
    const auto text = read_file(source_config(name));
    for (const auto & token : forbidden) {
      EXPECT_EQ(text.find(token), std::string::npos) << name << " contains " << token;
    }
  }
}

TEST(V2ProviderYaml, DetectedObservationAgeIsExplicitFinitePositivePolicy)
{
  const std::string base =
    "schema_version: 1\nprovider_id: detected\nobject_id: cube\ntarget_frame: world\n";
  EXPECT_DOUBLE_EQ(
    franka_motion::workflow_v2::parse_detected_observation_yaml(base).max_observation_age_sec,
    5.0);
  EXPECT_DOUBLE_EQ(
    franka_motion::workflow_v2::parse_detected_observation_yaml(
      base + "max_observation_age_sec: 12.5\n").max_observation_age_sec,
    12.5);
  for (const auto * value : {"0", "-1", ".nan", ".inf"}) {
    EXPECT_THROW(
      franka_motion::workflow_v2::parse_detected_observation_yaml(
        base + "max_observation_age_sec: " + value + "\n"),
      std::invalid_argument);
  }
}
