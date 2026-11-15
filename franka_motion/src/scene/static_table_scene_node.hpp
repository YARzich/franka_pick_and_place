// Responsibility: Declares the one-shot ROS node that inserts the table through the scene
//   coordinator.

#ifndef SCENE__STATIC_TABLE_SCENE_NODE_HPP_
#define SCENE__STATIC_TABLE_SCENE_NODE_HPP_

#include <string>

#include <manipulation_msgs/srv/acquire_planning_scene_lease.hpp>
#include <manipulation_msgs/srv/apply_planning_scene_diff.hpp>
#include <manipulation_msgs/srv/release_planning_scene_lease.hpp>
#include <rclcpp/rclcpp.hpp>

#include "manipulation/scene/box_collision.hpp"

namespace franka_motion::scene
{

/** Apply the simulator tabletop once through the PlanningScene coordinator.
 *
 * The node owns bootstrap orchestration only. PlanningScene serialization,
 * lease/revision policy and authoritative reconciliation remain in their
 * dedicated helpers and coordinator node.
 */
class StaticTableSceneNode : public rclcpp::Node
{
public:
  StaticTableSceneNode();

  /// Apply the configured table and return false on any unconfirmed outcome.
  [[nodiscard]] bool apply();

private:
  using AcquireLease = manipulation_msgs::srv::AcquirePlanningSceneLease;
  using ApplyCoordinatedDiff = manipulation_msgs::srv::ApplyPlanningSceneDiff;
  using ReleaseLease = manipulation_msgs::srv::ReleasePlanningSceneLease;

  void validate_configuration() const;
  [[nodiscard]] bool apply_coordinated();
  void log_success() const;
  [[nodiscard]] std::string resolve_operation_id() const;

  manipulation::scene::BoxCollisionSpec spec_;
  double service_timeout_sec_{30.0};
  double startup_retry_timeout_sec_{30.0};
  double startup_retry_period_sec_{0.25};
  double lease_ttl_sec_{60.0};
  std::string acquire_service_name_;
  std::string apply_service_name_;
  std::string release_service_name_;
  std::string owner_id_;
  std::string operation_id_;
  rclcpp::Client<AcquireLease>::SharedPtr acquire_client_;
  rclcpp::Client<ApplyCoordinatedDiff>::SharedPtr coordinated_apply_client_;
  rclcpp::Client<ReleaseLease>::SharedPtr release_client_;
};

}  // namespace franka_motion::scene

#endif  // SCENE__STATIC_TABLE_SCENE_NODE_HPP_
