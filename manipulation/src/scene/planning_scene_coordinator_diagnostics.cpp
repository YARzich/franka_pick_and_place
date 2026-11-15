// Responsibility: Publishes coordinator lease, backend, reconciliation and integrity diagnostics.

#include "planning_scene_coordinator_node.hpp"

#include <cinttypes>
#include <cstdint>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace manipulation::scene
{

void PlanningSceneCoordinatorNode::expire_leases()
{
  registry_->expire_leases();
  const auto snapshot = registry_->snapshot();

  std::lock_guard<std::mutex> lock(health_mutex_);
  if (snapshot.expired_lease_count <= last_reported_expiry_count_) {
    return;
  }
  const auto newly_expired = snapshot.expired_lease_count - last_reported_expiry_count_;
  last_reported_expiry_count_ = snapshot.expired_lease_count;
  unreconciled_expiry_count_ += newly_expired;
  reconciliation_required_ = true;
  last_backend_error_ = RegistryError::kLeaseExpired;
  last_backend_message_ =
    "lease expiry quarantined resource ownership; explicit recovery is required";

  // Expiry preserves fail-closed ownership without applying a blind inverse
  // diff, which could detach a physically held object or delete valid state.
  RCLCPP_WARN(
    get_logger(),
    "%" PRIu64 " PlanningScene lease(s) expired; resources quarantined without scene cleanup",
    newly_expired);
}

void PlanningSceneCoordinatorNode::record_backend_status(const RegistryStatus & status)
{
  // A request rejected by the deterministic scene model never reached MoveIt
  // and therefore must not degrade backend health.
  if (status.error == RegistryError::kInvalidRequest) {
    return;
  }
  std::lock_guard<std::mutex> lock(health_mutex_);
  if (reconciliation_required_ && status.success) {
    return;
  }
  last_backend_error_ = status.error;
  last_backend_message_ = status.message;
}

void PlanningSceneCoordinatorNode::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  expire_leases();
  const auto snapshot = registry_->snapshot();

  RegistryError backend_error;
  std::string backend_message;
  std::uint64_t reported_expiries;
  std::uint64_t unreconciled_expiries;
  bool backend_ready;
  bool reconciliation_required;
  const bool transform_ready = world_transform_ready();
  bool model_frame_verified;
  std::string observed_model_frame;
  std::string model_frame_message;
  {
    std::lock_guard<std::mutex> lock(model_frame_mutex_);
    model_frame_verified = backend_model_frame_verified_;
    observed_model_frame = observed_model_frame_;
    model_frame_message = model_frame_verification_message_;
  }
  {
    std::lock_guard<std::mutex> lock(health_mutex_);
    backend_error = last_backend_error_;
    backend_message = last_backend_message_;
    reported_expiries = last_reported_expiry_count_;
    unreconciled_expiries = unreconciled_expiry_count_;
    backend_ready = backend_service_ready_;
    reconciliation_required = reconciliation_required_;
  }

  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  if (!snapshot.quarantined_resource_keys.empty()) {
    status.summary(
      DiagnosticStatus::ERROR,
      "PlanningScene resources are quarantined; explicit recovery is required");
  } else if (reconciliation_required) {
    status.summary(
      DiagnosticStatus::ERROR,
      "PlanningScene reconciliation is required; mutations are blocked");
  } else if (snapshot.operation_capacity_exhausted) {
    status.summary(DiagnosticStatus::ERROR, "PlanningScene operation ledger is full");
  } else if (backend_error != RegistryError::kNone) {
    status.summary(DiagnosticStatus::ERROR, "last PlanningScene backend operation failed");
  } else if (reported_expiries > 0) {
    status.summary(
      DiagnosticStatus::WARN,
      "lease expiry requires authoritative scene reconciliation");
  } else if (!backend_ready) {
    status.summary(DiagnosticStatus::WARN, "MoveIt PlanningScene backend is unavailable");
  } else if (!transform_ready) {
    status.summary(
      DiagnosticStatus::WARN,
      "world-object mutations are waiting for the configured frame transform");
  } else if (!verify_fixed_root_model_frame_) {
    status.summary(
      DiagnosticStatus::WARN,
      "fixed-root MoveIt model-frame verification is disabled");
  } else {
    status.summary(DiagnosticStatus::OK, "PlanningScene coordinator is healthy");
  }

  status.add("session_id", snapshot.session_id);
  status.add("revision", snapshot.revision);
  status.add("active_leases", snapshot.active_lease_count);
  status.add("operation_records", snapshot.operation_record_count);
  status.add("operation_record_capacity", snapshot.operation_record_capacity);
  status.add("operation_capacity_exhausted", snapshot.operation_capacity_exhausted);
  status.add("operation_in_progress", snapshot.operation_in_progress);
  status.add("expired_leases_total", reported_expiries);
  status.add("unreconciled_lease_expiries", unreconciled_expiries);
  status.add("quarantined_resources", snapshot.quarantined_resource_keys.size());
  status.add("reconciliation_required", reconciliation_required);
  status.add("last_backend_error", to_interface_error(backend_error));
  status.add("last_backend_message", backend_message);
  status.add("backend_service", backend_service_name_);
  status.add("readback_service", readback_service_name_);
  status.add("planning_frame", planning_frame_);
  status.add("backend_planning_frame", backend_planning_frame_);
  status.add("verify_fixed_root_model_frame", verify_fixed_root_model_frame_);
  status.add("model_frame_verified", model_frame_verified);
  status.add("observed_model_frame", observed_model_frame);
  status.add("model_frame_verification", model_frame_message);
  status.add("world_transform_ready", transform_ready);
  status.add("backend_service_ready", backend_ready);
}

}  // namespace manipulation::scene
