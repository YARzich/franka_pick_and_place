// Responsibility: Constructs coordinator parameters, ROS interfaces, timers and shared state.

#include "planning_scene_coordinator_node.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "manipulation/validation/frame_id.hpp"

namespace manipulation::scene
{
namespace
{

std::chrono::steady_clock::duration positive_seconds(
  const double seconds,
  const std::string & parameter_name)
{
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(parameter_name + " must be finite and positive");
  }
  const auto duration = std::chrono::duration<double>{seconds};
  if (duration > std::chrono::duration<double>(std::chrono::steady_clock::duration::max())) {
    throw std::invalid_argument(parameter_name + " exceeds steady-clock range");
  }
  const auto result =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(duration);
  if (result <= std::chrono::steady_clock::duration::zero()) {
    throw std::invalid_argument(parameter_name + " is below steady-clock resolution");
  }
  return result;
}

}  // namespace

PlanningSceneCoordinatorNode::PlanningSceneCoordinatorNode(
  const rclcpp::NodeOptions & options)
: Node("planning_scene_coordinator", options), diagnostics_(this)
{
  backend_service_name_ = declare_parameter<std::string>(
    "backend_service", "/apply_planning_scene");
  if (backend_service_name_.empty()) {
    throw std::invalid_argument("backend_service must not be empty");
  }
  readback_service_name_ = declare_parameter<std::string>(
    "readback_service", "/get_planning_scene");
  if (readback_service_name_.empty()) {
    throw std::invalid_argument("readback_service must not be empty");
  }
  planning_frame_ = declare_parameter<std::string>("planning_frame", "world");
  if (!manipulation::validation::is_canonical_relative_frame_id(planning_frame_)) {
    throw std::invalid_argument("planning_frame must be a canonical relative frame ID");
  }
  backend_planning_frame_ = declare_parameter<std::string>(
    "backend_planning_frame", "world");
  if (!manipulation::validation::is_canonical_relative_frame_id(
      backend_planning_frame_))
  {
    throw std::invalid_argument(
            "backend_planning_frame must be a canonical relative frame ID");
  }
  robot_description_topic_ = declare_parameter<std::string>(
    "robot_description_topic", "/robot_description");
  verify_fixed_root_model_frame_ = declare_parameter<bool>(
    "verify_fixed_root_model_frame", true);
  if (verify_fixed_root_model_frame_ && robot_description_topic_.empty()) {
    throw std::invalid_argument(
            "robot_description_topic must not be empty when frame verification is enabled");
  }
  backend_service_timeout_ = positive_seconds(
    declare_parameter<double>("backend_service_timeout_sec", 5.0),
    "backend_service_timeout_sec");
  readback_service_timeout_ = positive_seconds(
    declare_parameter<double>("readback_service_timeout_sec", 5.0),
    "readback_service_timeout_sec");
  const auto expiry_period = positive_seconds(
    declare_parameter<double>("lease_expiry_check_period_sec", 0.5),
    "lease_expiry_check_period_sec");
  const auto reconciliation_period = positive_seconds(
    declare_parameter<double>("reconciliation_check_period_sec", 1.0),
    "reconciliation_check_period_sec");

  const auto initial_revision = declare_parameter<std::int64_t>("initial_revision", 0);
  const auto max_operation_records = declare_parameter<std::int64_t>(
    "max_operation_records", 4096);
  if (initial_revision < 0) {
    throw std::invalid_argument("initial_revision must be non-negative");
  }
  if (max_operation_records <= 0 ||
    static_cast<std::uint64_t>(max_operation_records) >
    std::numeric_limits<std::uint32_t>::max())
  {
    throw std::invalid_argument("max_operation_records is outside uint32 interface range");
  }

  auto session_id = declare_parameter<std::string>("session_id", "");
  if (session_id.empty()) {
    session_id = generate_session_id();
  }
  SceneLeaseRegistry::Options registry_options;
  registry_options.session_id = session_id;
  registry_options.initial_revision = static_cast<std::uint64_t>(initial_revision);
  registry_options.max_operation_records = static_cast<std::size_t>(max_operation_records);
  registry_ = std::make_unique<SceneLeaseRegistry>(std::move(registry_options));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
    *tf_buffer_, get_node_base_interface(), get_node_logging_interface(),
    get_node_parameters_interface(), get_node_topics_interface(), false);
  if (verify_fixed_root_model_frame_) {
    robot_description_subscription_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(
        &PlanningSceneCoordinatorNode::handle_robot_description, this,
        std::placeholders::_1));
  } else {
    std::lock_guard<std::mutex> lock(model_frame_mutex_);
    backend_model_frame_verified_ = true;
    model_frame_verification_message_ =
      "fixed-root model-frame verification explicitly disabled";
  }

  // Metadata services remain responsive while one scene-I/O callback waits for
  // MoveIt. The mutually-exclusive group is the admission gate that prevents
  // executor threads from piling up behind backend_call_mutex_.
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  scene_io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  backend_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  maintenance_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  backend_client_ = create_client<ApplyPlanningScene>(
    backend_service_name_, rclcpp::ServicesQoS(), backend_group_);
  readback_client_ = create_client<GetPlanningScene>(
    readback_service_name_, rclcpp::ServicesQoS(), backend_group_);

  acquire_service_ = create_service<AcquireLease>(
    "~/acquire_lease",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_acquire, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
  renew_service_ = create_service<RenewLease>(
    "~/renew_lease",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_renew, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
  release_service_ = create_service<ReleaseLease>(
    "~/release_lease",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_release, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
  validate_service_ = create_service<ValidateLease>(
    "~/validate_lease",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_validate, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), scene_io_group_);
  apply_service_ = create_service<ApplySceneDiff>(
    "~/apply_scene_diff",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_apply, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), scene_io_group_);
  state_service_ = create_service<GetSceneState>(
    "~/get_state",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_get_state, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
  reconcile_service_ = create_service<ReconcileScene>(
    "~/reconcile",
    std::bind(
      &PlanningSceneCoordinatorNode::handle_reconcile, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), scene_io_group_);

  expiry_timer_ = create_wall_timer(
    expiry_period,
    std::bind(&PlanningSceneCoordinatorNode::expire_leases, this),
    maintenance_group_);
  reconciliation_timer_ = create_wall_timer(
    reconciliation_period,
    std::bind(&PlanningSceneCoordinatorNode::maintain_consistency, this),
    scene_io_group_);
  diagnostics_.setHardwareID(session_id);
  diagnostics_.add(
    "PlanningScene coordinator", this,
    &PlanningSceneCoordinatorNode::produce_diagnostics);

  RCLCPP_INFO(
    get_logger(),
    "PlanningScene coordinator session=%s backend=%s readback=%s frames=%s->%s",
    session_id.c_str(), backend_service_name_.c_str(), readback_service_name_.c_str(),
    planning_frame_.c_str(), backend_planning_frame_.c_str());
}

}  // namespace manipulation::scene
