// Responsibility: Acquires table ownership, applies its collision geometry and releases the lease
//   safely.

#include "static_table_scene_node.hpp"

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <manipulation_msgs/msg/planning_scene_operation_result.hpp>

#include "manipulation/scene/resource_key.hpp"
#include "manipulation/scene/scene_client_support.hpp"

namespace franka_motion::scene
{
using manipulation::scene::duration_from_positive_seconds;
using manipulation::scene::is_valid;
using manipulation::scene::LeaseReleaseGuard;
using manipulation::scene::make_planning_scene_diff;
using manipulation::scene::make_scene_operation_id;
using manipulation::scene::make_world_resource_key;
using manipulation::scene::parse_vector3;

namespace
{

template<typename ServiceT>
typename ServiceT::Response::SharedPtr call_service(
  const rclcpp::Node::SharedPtr & node,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request,
  const std::chrono::duration<double> timeout)
{
  if (!client->wait_for_service(timeout)) {
    return nullptr;
  }
  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(node, future, timeout);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(future);
    return nullptr;
  }
  return future.get();
}

}  // namespace

StaticTableSceneNode::StaticTableSceneNode()
: Node("static_table_scene")
{
  spec_.object_id = declare_parameter<std::string>("object_id", "table");
  spec_.frame_id = declare_parameter<std::string>("frame_id", "panda_link0");
  spec_.dimensions = parse_vector3(
    declare_parameter<std::vector<double>>("dimensions", {1.2, 1.2, 0.05}),
    "dimensions");
  spec_.position = parse_vector3(
    declare_parameter<std::vector<double>>("position", {0.4, 0.0, -0.025}),
    "position");
  service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 30.0);
  startup_retry_timeout_sec_ = declare_parameter<double>(
    "startup_retry_timeout_sec", 30.0);
  startup_retry_period_sec_ = declare_parameter<double>(
    "startup_retry_period_sec", 0.25);
  const bool use_coordinator = declare_parameter<bool>("use_scene_coordinator", true);
  acquire_service_name_ = declare_parameter<std::string>(
    "coordinator_acquire_service", "/planning_scene_coordinator/acquire_lease");
  apply_service_name_ = declare_parameter<std::string>(
    "coordinator_apply_service", "/planning_scene_coordinator/apply_scene_diff");
  release_service_name_ = declare_parameter<std::string>(
    "coordinator_release_service", "/planning_scene_coordinator/release_lease");
  owner_id_ = declare_parameter<std::string>(
    "coordinator_owner_id", "static_table_scene");
  lease_ttl_sec_ = declare_parameter<double>("lease_ttl_sec", 60.0);
  operation_id_ = declare_parameter<std::string>("operation_id", "");

  if (!use_coordinator) {
    throw std::invalid_argument(
            "canonical static_table_scene is coordinator-only; use the "
            "franka_motion compatibility executable for legacy direct mode");
  }
  validate_configuration();
  acquire_client_ = create_client<AcquireLease>(acquire_service_name_);
  coordinated_apply_client_ = create_client<ApplyCoordinatedDiff>(apply_service_name_);
  release_client_ = create_client<ReleaseLease>(release_service_name_);
}

bool StaticTableSceneNode::apply()
{
  return apply_coordinated();
}

void StaticTableSceneNode::validate_configuration() const
{
  if (!is_valid(spec_)) {
    throw std::invalid_argument("Table collision parameters are invalid");
  }
  const auto finite_positive = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
  if (!finite_positive(service_timeout_sec_)) {
    throw std::invalid_argument("service_timeout_sec must be finite and positive");
  }
  if (!finite_positive(startup_retry_timeout_sec_) ||
    !finite_positive(startup_retry_period_sec_))
  {
    throw std::invalid_argument("startup retry timeout and period must be finite and positive");
  }
  if (owner_id_.empty()) {
    throw std::invalid_argument("coordinator_owner_id must not be empty");
  }
  (void)duration_from_positive_seconds(lease_ttl_sec_);
  if (lease_ttl_sec_ <= startup_retry_timeout_sec_) {
    throw std::invalid_argument("lease_ttl_sec must exceed startup_retry_timeout_sec");
  }
}

bool StaticTableSceneNode::apply_coordinated()
{
  using Result = manipulation_msgs::msg::PlanningSceneOperationResult;
  const auto resource_key = make_world_resource_key(spec_.object_id);
  if (!resource_key) {
    RCLCPP_ERROR(get_logger(), "Table object ID cannot form a resource key");
    return false;
  }

  const auto service_timeout = std::chrono::duration<double>(service_timeout_sec_);
  const auto acquire_request = std::make_shared<AcquireLease::Request>();
  acquire_request->owner_id = owner_id_;
  acquire_request->resource_keys = {*resource_key};
  acquire_request->requested_ttl = duration_from_positive_seconds(lease_ttl_sec_);
  const auto acquired = call_service<AcquireLease>(
    shared_from_this(), acquire_client_, acquire_request, service_timeout);
  if (!acquired || !acquired->result.success) {
    RCLCPP_ERROR(get_logger(), "Failed to acquire PlanningScene table lease");
    return false;
  }

  const auto lease_id = acquired->lease_id;
  LeaseReleaseGuard release_guard(
    [this, lease_id, service_timeout]() {
      const auto request = std::make_shared<ReleaseLease::Request>();
      request->lease_id = lease_id;
      const auto response = call_service<ReleaseLease>(
        shared_from_this(), release_client_, request, service_timeout);
      if (!response || !response->result.success) {
        RCLCPP_ERROR(get_logger(), "Failed to release PlanningScene table lease");
        return false;
      }
      return true;
    });

  const auto operation_id = resolve_operation_id();
  const auto expected_revision = acquired->result.revision;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(startup_retry_timeout_sec_));
  bool retry_reported = false;
  while (rclcpp::ok()) {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    if (remaining <= std::chrono::steady_clock::duration::zero()) {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for initial PlanningScene reconciliation");
      return false;
    }

    const auto request = std::make_shared<ApplyCoordinatedDiff::Request>();
    request->lease_id = lease_id;
    request->operation_id = operation_id;
    request->expected_revision = expected_revision;
    request->resource_keys = {*resource_key};
    request->scene_diff = make_planning_scene_diff(spec_);
    const auto response = call_service<ApplyCoordinatedDiff>(
      shared_from_this(), coordinated_apply_client_, request,
      std::min(service_timeout, std::chrono::duration<double>(remaining)));
    if (!response) {
      RCLCPP_ERROR(get_logger(), "PlanningScene coordinator did not answer the table diff");
      return false;
    }
    if (response->result.success) {
      if (!release_guard.release()) {
        RCLCPP_ERROR(get_logger(), "PlanningScene table lease remains unreleased");
        return false;
      }
      log_success();
      return true;
    }

    // These typed results are emitted only before backend dispatch. Retrying
    // any ambiguous or post-mutation outcome would violate the single-writer
    // and idempotency contract.
    const bool safe_startup_retry =
      (response->result.error_code == Result::RECONCILIATION_REQUIRED ||
      response->result.error_code == Result::DEPENDENCY_UNAVAILABLE) &&
      response->result.revision == expected_revision;
    if (!safe_startup_retry) {
      RCLCPP_ERROR(
        get_logger(),
        "PlanningScene coordinator rejected the table diff: "
        "code=%u revision=%" PRIu64 " message=%s",
        static_cast<unsigned int>(response->result.error_code),
        response->result.revision,
        response->result.message.c_str());
      return false;
    }
    if (!retry_reported) {
      RCLCPP_INFO(
        get_logger(), "Waiting for PlanningScene startup dependencies");
      retry_reported = true;
    }
    const auto remaining_after_response = deadline - std::chrono::steady_clock::now();
    if (remaining_after_response > std::chrono::steady_clock::duration::zero()) {
      const auto retry_period = std::chrono::duration<double>(startup_retry_period_sec_);
      std::this_thread::sleep_for(
        std::min(retry_period, std::chrono::duration<double>(remaining_after_response)));
    }
  }
  RCLCPP_ERROR(get_logger(), "ROS shutdown interrupted table scene bootstrap");
  return false;
}

void StaticTableSceneNode::log_success() const
{
  RCLCPP_INFO(
    get_logger(), "Added collision object '%s' in frame '%s'",
    spec_.object_id.c_str(), spec_.frame_id.c_str());
}

std::string StaticTableSceneNode::resolve_operation_id() const
{
  if (!operation_id_.empty()) {
    return operation_id_;
  }
  std::random_device entropy;
  return make_scene_operation_id(
    owner_id_, "add", spec_.object_id,
    static_cast<std::uint32_t>(entropy()),
    static_cast<std::uint32_t>(entropy()));
}

}  // namespace franka_motion::scene
