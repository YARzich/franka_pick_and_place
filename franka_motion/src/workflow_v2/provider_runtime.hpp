// Responsibility: Owns the ROS resources and concrete providers selected for one V2 process.

#ifndef WORKFLOW_V2__PROVIDER_RUNTIME_HPP_
#define WORKFLOW_V2__PROVIDER_RUNTIME_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "workflow_v2/detected_pose_buffer.hpp"
#include "workflow_v2/mtc_grasp_provider.hpp"
#include "workflow_v2/process_config.hpp"

namespace franka_motion::workflow_v2
{

/**
 * Keep provider-side ROS and MoveIt state alive for a future composition root.
 *
 * The retained node must be spun by the outer executor. The runtime owns either
 * a configured provider or the detected subscription/buffer/exact-time TF path,
 * plus the production FixedState MTC provider and its robot-model reference.
 */
class ProviderRuntime final
{
public:
  ProviderRuntime(
    rclcpp::Node::SharedPtr node,
    const LoadedV2ProcessConfig & config,
    moveit::core::RobotModelConstPtr robot_model);
  ~ProviderRuntime();

  ProviderRuntime(const ProviderRuntime &) = delete;
  ProviderRuntime & operator=(const ProviderRuntime &) = delete;
  ProviderRuntime(ProviderRuntime &&) = delete;
  ProviderRuntime & operator=(ProviderRuntime &&) = delete;

  [[nodiscard]] const manipulation::workflow::ObjectCatalog & object_catalog() const noexcept;
  [[nodiscard]] manipulation::workflow::ObservationProvider & observation_provider() noexcept;
  [[nodiscard]] manipulation::workflow::GraspProvider & grasp_provider() noexcept;
  [[nodiscard]] const moveit::core::RobotModelConstPtr & robot_model() const noexcept;
  [[nodiscard]] bool owns_detected_transport() const noexcept;

  void request_stop() noexcept;

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  manipulation::workflow::ObjectCatalog object_catalog_;

  // Declaration order preserves callback dependency lifetimes during teardown.
  std::unique_ptr<DetectedPoseBuffer> detected_pose_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detected_pose_subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<manipulation::workflow::ObservationProvider> observation_provider_;
  std::unique_ptr<MtcGraspProvider> grasp_provider_;
};

}  // namespace franka_motion::workflow_v2

#endif  // WORKFLOW_V2__PROVIDER_RUNTIME_HPP_
