// Responsibility: Constructs and exposes the production V2 ROS scene adapter facades.

#include "manipulation_moveit/ros_v2_scene_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "manipulation/validation/frame_id.hpp"

#include "ros_v2_scene_adapter_impl.hpp"

namespace manipulation::moveit
{
namespace
{

bool valid_identifier(const std::string & value)
{
  return !value.empty() && std::all_of(
    value.cbegin(), value.cend(), [](const unsigned char character) {
      return std::isspace(character) == 0 && std::iscntrl(character) == 0;
    });
}

}  // namespace

void validateRosV2SceneAdapterConfig(const RosV2SceneAdapterConfig & config)
{
  if (!valid_identifier(config.owner_id) ||
    !manipulation::validation::is_canonical_relative_frame_id(config.attachment_link) ||
    config.touch_links.empty())
  {
    throw std::invalid_argument(
            "V2 scene owner, attachment link, and touch links must be configured");
  }
  std::unordered_set<std::string> unique_links;
  for (const auto & link : config.touch_links) {
    if (!manipulation::validation::is_canonical_relative_frame_id(link) ||
      !unique_links.insert(link).second)
    {
      throw std::invalid_argument("V2 scene touch links must be canonical and unique");
    }
  }
  if (unique_links.count(config.attachment_link) == 0U) {
    throw std::invalid_argument("V2 scene touch links must include the attachment link");
  }
  const std::string * endpoints[] = {
    &config.acquire_service, &config.renew_service, &config.release_service,
    &config.validate_service, &config.state_service, &config.apply_diff_service,
    &config.get_planning_scene_service};
  if (std::any_of(
      std::cbegin(endpoints), std::cend(endpoints),
      [](const auto * endpoint) {return endpoint->empty();}))
  {
    throw std::invalid_argument("V2 scene service endpoints must be non-empty");
  }
}

RosV2SceneAdapter::Impl::Impl(
  rclcpp::Node & node,
  ::moveit::core::RobotModelConstPtr robot_model,
  RosV2SceneAdapterConfig config,
  const manipulation::workflow::MeshResourceLoader & mesh_loader)
: config_(std::move(config)), robot_model_(std::move(robot_model)),
  mesh_loader_(mesh_loader),
  context_(node.get_node_base_interface()->get_context()),
  callback_group_(node.create_callback_group(rclcpp::CallbackGroupType::Reentrant))
{
  validateRosV2SceneAdapterConfig(config_);
  if (!robot_model_) {
    throw std::invalid_argument("V2 scene adapter requires a robot model");
  }
  acquire_client_ = node.create_client<Acquire>(
    config_.acquire_service, rclcpp::ServicesQoS(), callback_group_);
  renew_client_ = node.create_client<Renew>(
    config_.renew_service, rclcpp::ServicesQoS(), callback_group_);
  release_client_ = node.create_client<Release>(
    config_.release_service, rclcpp::ServicesQoS(), callback_group_);
  validate_client_ = node.create_client<Validate>(
    config_.validate_service, rclcpp::ServicesQoS(), callback_group_);
  state_client_ = node.create_client<GetState>(
    config_.state_service, rclcpp::ServicesQoS(), callback_group_);
  apply_client_ = node.create_client<Apply>(
    config_.apply_diff_service, rclcpp::ServicesQoS(), callback_group_);
  get_scene_client_ = node.create_client<GetScene>(
    config_.get_planning_scene_service, rclcpp::ServicesQoS(), callback_group_);
}

void RosV2SceneAdapter::Impl::begin_goal() noexcept
{
  const std::lock_guard<std::mutex> operation_lock(operation_mutex_);
  stop_state_.begin_goal();
}

void RosV2SceneAdapter::Impl::request_stop() noexcept
{
  const std::lock_guard<std::mutex> operation_lock(operation_mutex_);
  stop_state_.request_stop();
}

PickLiftPlaceReadiness RosV2SceneAdapter::Impl::cached_readiness() const
{
  if (!context_ || !context_->is_valid()) {
    return {false, "V2 scene adapter ROS context is shutting down"};
  }
  if (!acquire_client_->service_is_ready() ||
    !renew_client_->service_is_ready() ||
    !release_client_->service_is_ready() ||
    !validate_client_->service_is_ready() ||
    !state_client_->service_is_ready() ||
    !apply_client_->service_is_ready() ||
    !get_scene_client_->service_is_ready())
  {
    return {false, "one or more coordinator/full-scene services are unavailable"};
  }
  return {true, "all coordinator/full-scene services are discoverable"};
}

RosV2SceneAdapter::RosV2SceneAdapter(
  rclcpp::Node & node,
  ::moveit::core::RobotModelConstPtr robot_model,
  RosV2SceneAdapterConfig config)
: owned_mesh_loader_(std::make_unique<GeometricShapesMeshResourceLoader>()),
  impl_(std::make_unique<Impl>(
      node, std::move(robot_model), std::move(config), *owned_mesh_loader_))
{
}

RosV2SceneAdapter::RosV2SceneAdapter(
  rclcpp::Node & node,
  ::moveit::core::RobotModelConstPtr robot_model,
  RosV2SceneAdapterConfig config,
  const manipulation::workflow::MeshResourceLoader & mesh_loader)
: impl_(std::make_unique<Impl>(
      node, std::move(robot_model), std::move(config), mesh_loader))
{
}

RosV2SceneAdapter::~RosV2SceneAdapter() = default;

PickLiftPlaceReadiness RosV2SceneAdapter::cached_readiness() const
{
  return impl_->cached_readiness();
}

manipulation::workflow::PlanningSceneLeaseResult RosV2SceneAdapter::acquire(
  const std::vector<manipulation::workflow::ObjectInstance> & instances,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->acquire(instances, context);
}

manipulation::workflow::PlanningSceneAccessProofResult RosV2SceneAdapter::renew(
  const manipulation::workflow::PlanningSceneAccessProof & access_proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->renew(access_proof, context);
}

manipulation::workflow::PlanningSceneLeaseResult RosV2SceneAdapter::stage_observation(
  const manipulation::workflow::ObjectInstance & instance,
  const manipulation::workflow::ObjectProfile & profile,
  const manipulation::workflow::ObjectObservation & observation,
  const manipulation::workflow::PlanningSceneAccessProof & access_proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->stage_observation(
    instance, profile, observation, access_proof, context);
}

manipulation::workflow::PlanningSceneLeaseResult RosV2SceneAdapter::revalidate(
  const manipulation::workflow::PlanningSceneAccessProof & access_proof,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->revalidate(access_proof, context);
}

void RosV2SceneAdapter::release(
  const manipulation::workflow::PlanningSceneAccessProof & access_proof) noexcept
{
  impl_->release(access_proof);
}

void RosV2SceneAdapter::begin_goal()
{
  impl_->begin_goal();
}

CoordinatedSceneResult RosV2SceneAdapter::revalidate(
  const StrategySceneRevalidationRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->strategy_revalidate(request, context);
}

CoordinatedSceneResult RosV2SceneAdapter::attach(
  const StrategySceneAttachRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->attach(request, context);
}

CoordinatedSceneResult RosV2SceneAdapter::detach(
  const StrategySceneDetachRequest & request,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->detach(request, context);
}

SceneRevisionCheck RosV2SceneAdapter::revalidate(
  const PlanningSceneRevision & expected,
  const PlanningStopToken & stop_token)
{
  return impl_->planning_revalidate(expected, stop_token);
}

void RosV2SceneAdapter::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::moveit
