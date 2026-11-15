// Responsibility: Provides deterministic valid domain values for isolated transfer-contract tests.

#ifndef WORKFLOW__TRANSFER_DOMAIN_TEST_SUPPORT_HPP_
#define WORKFLOW__TRANSFER_DOMAIN_TEST_SUPPORT_HPP_

#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"

#include "manipulation/workflow/transfer_workflow_types.hpp"

namespace manipulation::workflow::test_support
{

inline builtin_interfaces::msg::Time test_stamp()
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123U;
  return stamp;
}

inline geometry_msgs::msg::Pose rigid_pose(
  const double x = 0.0,
  const double y = 0.0,
  const double z = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

inline geometry_msgs::msg::PoseStamped stamped_pose(
  const std::string & frame_id = "world",
  const double x = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = test_stamp();
  pose.pose = rigid_pose(x);
  return pose;
}

inline ObjectInstance instance()
{
  return ObjectInstance::create("cube_1", "cube_profile");
}

inline ObjectProfile profile()
{
  return ObjectProfile::create(
    "cube_profile",
    ObjectGeometry::primitive(
      PrimitiveGeometry::create(
        PrimitiveGeometryType::kBox, {0.1, 0.2, 0.3}, rigid_pose())));
}

inline ObjectObservation observation(const std::string & correlation_id = "observation-1")
{
  return ObjectObservation::create("cube_1", stamped_pose(), correlation_id, 0.9);
}

inline GraspCandidate grasp_candidate(
  const ObjectObservation & source,
  const double score = 0.8,
  const double opening = 0.04)
{
  auto pregrasp = source.pose();
  pregrasp.pose.position.x -= 0.1;
  auto grasp = source.pose();
  geometry_msgs::msg::Vector3Stamped approach;
  approach.header = source.pose().header;
  approach.vector.x = 1.0;
  return GraspCandidate::create(
    source.correlation(), std::move(pregrasp), std::move(grasp), std::move(approach),
    opening, 20.0, score);
}

inline PlanningSceneSnapshot scene_snapshot(
  const std::uint64_t revision = 7U,
  const std::string & session_id = "scene-session")
{
  moveit_msgs::msg::PlanningScene scene;
  scene.name = "test-scene";
  scene.is_diff = false;
  return PlanningSceneSnapshot::create(session_id, revision, std::move(scene));
}

inline PlanningSceneAccessProof scene_access_proof(
  const std::uint64_t revision = 7U,
  const std::string & session_id = "scene-session")
{
  return PlanningSceneAccessProof::create("lease-1", session_id, revision);
}

inline ResolvedTransferTask resolved_task()
{
  auto observed = observation();
  auto grasp = grasp_candidate(observed);
  return ResolvedTransferTask::create(
    instance(), profile(), std::move(observed), std::move(grasp), scene_snapshot(),
    scene_access_proof(), stamped_pose("world", 0.5), true,
    TransferRecoveryPolicy::kRetreatAndHome);
}

}  // namespace manipulation::workflow::test_support

#endif  // WORKFLOW__TRANSFER_DOMAIN_TEST_SUPPORT_HPP_
