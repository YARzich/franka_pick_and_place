// Responsibility: Defines parameter names that publish an authoritative MoveIt snapshot manifest.

#ifndef FRANKA_MOTION__MOVEIT_PARAMETER_SNAPSHOT_CONTRACT_HPP_
#define FRANKA_MOTION__MOVEIT_PARAMETER_SNAPSHOT_CONTRACT_HPP_

namespace franka_motion
{

/**
 * String-array parameter listing the immutable MoveIt prefix snapshot.
 *
 * This process-local name is private to the immutable startup snapshot helper.
 */
inline constexpr char kMoveItParameterSnapshotManifest[] =
  "franka_motion_moveit.parameter_snapshot_manifest";

}  // namespace franka_motion

#endif  // FRANKA_MOTION__MOVEIT_PARAMETER_SNAPSHOT_CONTRACT_HPP_
