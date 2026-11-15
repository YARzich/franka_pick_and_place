// Responsibility: Construct selected observation transport and production FixedState MTC provider.

#include "workflow_v2/provider_runtime.hpp"

#include <stdexcept>
#include <utility>

namespace franka_motion::workflow_v2
{
namespace
{

rclcpp::Node::SharedPtr require_node(rclcpp::Node::SharedPtr node)
{
  if (!node) {
    throw std::invalid_argument("V2 provider runtime requires a ROS node");
  }
  return node;
}

moveit::core::RobotModelConstPtr require_robot_model(
  moveit::core::RobotModelConstPtr robot_model)
{
  if (!robot_model) {
    throw std::invalid_argument("V2 provider runtime requires a robot model");
  }
  return robot_model;
}

}  // namespace

ProviderRuntime::ProviderRuntime(
  rclcpp::Node::SharedPtr node,
  const LoadedV2ProcessConfig & config,
  moveit::core::RobotModelConstPtr robot_model)
: node_(require_node(std::move(node))),
  robot_model_(require_robot_model(std::move(robot_model))),
  object_catalog_(config.object_catalog)
{
  if (config.workflow.grasp_provider_id != kMtcGraspProviderId) {
    throw std::invalid_argument("V2 provider runtime supports only the MTC grasp provider");
  }

  if (config.workflow.observation_provider_id == kConfiguredObservationProviderId) {
    const auto * preset = std::get_if<ConfiguredObservationPreset>(
      &config.observation_provider);
    if (!preset) {
      throw std::invalid_argument("configured provider ID does not match loaded provider config");
    }
    const auto clock = node_->get_clock();
    observation_provider_ = std::make_unique<ConfiguredObservationProvider>(
      *preset, [clock]() {
        return static_cast<builtin_interfaces::msg::Time>(clock->now());
      });
  } else if (config.workflow.observation_provider_id == kDetectedObservationProviderId) {
    const auto * detected = std::get_if<DetectedObservationProviderConfig>(
      &config.observation_provider);
    if (!detected) {
      throw std::invalid_argument("detected provider ID does not match loaded provider config");
    }
    detected_pose_buffer_ = std::make_unique<DetectedPoseBuffer>();
    detected_pose_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      config.deployment.detected_pose_topic,
      rclcpp::QoS(5).reliable(),
      [buffer = detected_pose_buffer_.get()](
        const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        buffer->updateCubePose(*message);
      });
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, node_, false);
    observation_provider_ = std::make_unique<DetectedObservationProvider>(
      *detected_pose_buffer_, tf_buffer_, *detected,
      [clock = node_->get_clock()]() {
        return static_cast<builtin_interfaces::msg::Time>(clock->now());
      });
  } else {
    throw std::invalid_argument("V2 provider runtime received an unknown observation provider");
  }

  auto planner = std::make_unique<FixedStateMtcSnapshotPlanner>(
    robot_model_, config.robot.end_effector_link);
  grasp_provider_ = std::make_unique<MtcGraspProvider>(
    config.grasp_profiles, std::move(planner));
}

ProviderRuntime::~ProviderRuntime() = default;

const manipulation::workflow::ObjectCatalog & ProviderRuntime::object_catalog() const noexcept
{
  return object_catalog_;
}

manipulation::workflow::ObservationProvider & ProviderRuntime::observation_provider() noexcept
{
  return *observation_provider_;
}

manipulation::workflow::GraspProvider & ProviderRuntime::grasp_provider() noexcept
{
  return *grasp_provider_;
}

const moveit::core::RobotModelConstPtr & ProviderRuntime::robot_model() const noexcept
{
  return robot_model_;
}

bool ProviderRuntime::owns_detected_transport() const noexcept
{
  return detected_pose_buffer_ && detected_pose_subscription_ && tf_buffer_ && tf_listener_;
}

void ProviderRuntime::request_stop() noexcept
{
  if (observation_provider_) {
    observation_provider_->request_stop();
  }
  if (grasp_provider_) {
    grasp_provider_->request_stop();
  }
}

}  // namespace franka_motion::workflow_v2
