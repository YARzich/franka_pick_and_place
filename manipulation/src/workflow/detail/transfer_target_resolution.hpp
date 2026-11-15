// Responsibility: Declare the private pure transfer-target geometry policy.

#ifndef WORKFLOW__DETAIL__TRANSFER_TARGET_RESOLUTION_HPP_
#define WORKFLOW__DETAIL__TRANSFER_TARGET_RESOLUTION_HPP_

#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "manipulation/workflow/transfer_workflow_types.hpp"
#include "manipulation_msgs/msg/transfer_object_step_v2.hpp"

namespace manipulation::workflow::detail
{

/** Resolve an admitted target without mutating source observations or profile geometry. */
[[nodiscard]] geometry_msgs::msg::PoseStamped resolve_transfer_target(
  const manipulation_msgs::msg::TransferObjectStepV2 & step,
  const ObjectObservation & observation,
  const ObjectProfile & profile,
  const std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> & initial_sources);

}  // namespace manipulation::workflow::detail

#endif  // WORKFLOW__DETAIL__TRANSFER_TARGET_RESOLUTION_HPP_
