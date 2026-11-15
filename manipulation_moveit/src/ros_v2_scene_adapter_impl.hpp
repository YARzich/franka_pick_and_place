// Responsibility: Owns V2 coordinator clients, goal stop generation, and active lease state.

#ifndef ROS_V2_SCENE_ADAPTER_IMPL_HPP_
#define ROS_V2_SCENE_ADAPTER_IMPL_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "manipulation_msgs/srv/acquire_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/apply_planning_scene_diff.hpp"
#include "manipulation_msgs/srv/get_planning_scene_state.hpp"
#include "manipulation_msgs/srv/release_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/renew_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/validate_planning_scene_lease.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"

#include "manipulation_moveit/ros_v2_scene_adapter.hpp"

#include "v2_scene_adapter_support.hpp"

namespace manipulation::moveit
{

class RosV2SceneAdapter::Impl final
{
public:
  using Clock = std::chrono::steady_clock;
  using Acquire = manipulation_msgs::srv::AcquirePlanningSceneLease;
  using Apply = manipulation_msgs::srv::ApplyPlanningSceneDiff;
  using GetScene = moveit_msgs::srv::GetPlanningScene;
  using GetState = manipulation_msgs::srv::GetPlanningSceneState;
  using Release = manipulation_msgs::srv::ReleasePlanningSceneLease;
  using Renew = manipulation_msgs::srv::RenewPlanningSceneLease;
  using Validate = manipulation_msgs::srv::ValidatePlanningSceneLease;

  Impl(
    rclcpp::Node & node,
    ::moveit::core::RobotModelConstPtr robot_model,
    RosV2SceneAdapterConfig config,
    const manipulation::workflow::MeshResourceLoader & mesh_loader);

  void begin_goal() noexcept;
  void request_stop() noexcept;
  [[nodiscard]] PickLiftPlaceReadiness cached_readiness() const;

  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult acquire(
    const std::vector<manipulation::workflow::ObjectInstance> & instances,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] manipulation::workflow::PlanningSceneAccessProofResult renew(
    const manipulation::workflow::PlanningSceneAccessProof & proof,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult stage_observation(
    const manipulation::workflow::ObjectInstance & instance,
    const manipulation::workflow::ObjectProfile & profile,
    const manipulation::workflow::ObjectObservation & observation,
    const manipulation::workflow::PlanningSceneAccessProof & proof,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] manipulation::workflow::PlanningSceneLeaseResult revalidate(
    const manipulation::workflow::PlanningSceneAccessProof & proof,
    manipulation::workflow::TransferExecutionContext & context);
  void release(const manipulation::workflow::PlanningSceneAccessProof & proof) noexcept;

  [[nodiscard]] CoordinatedSceneResult strategy_revalidate(
    const StrategySceneRevalidationRequest & request,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] CoordinatedSceneResult attach(
    const StrategySceneAttachRequest & request,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] CoordinatedSceneResult detach(
    const StrategySceneDetachRequest & request,
    manipulation::workflow::TransferExecutionContext & context);
  [[nodiscard]] SceneRevisionCheck planning_revalidate(
    const PlanningSceneRevision & expected,
    const PlanningStopToken & stop_token);

private:
  static constexpr Clock::duration kWaitSlice{std::chrono::milliseconds{20}};
  static constexpr Clock::duration kLeaseTtl{std::chrono::seconds{45}};
  static constexpr Clock::duration kExpectedRenewPeriod{std::chrono::seconds{10}};

  enum class FailureKind : std::uint8_t
  {
    kCanceled,
    kShutdown,
    kConflict,
    kUnavailable,
    kInvalid,
    kFailed,
    kStateUncertain,
    kInternal,
  };

  enum class ServiceStatus : std::uint8_t
  {
    kResponse,
    kStopped,
    kShutdown,
    kFailed,
  };

  template<typename ServiceT>
  struct ServiceCall
  {
    ServiceStatus status{ServiceStatus::kFailed};
    typename ServiceT::Response::SharedPtr response;
    std::string detail;
    bool request_sent{false};
  };

  struct WaitControl
  {
    std::function<bool()> stopped;
    std::function<detail::InterruptibleWaitResult()> wait_one_slice;
    std::function<detail::InterruptibleWaitResult()> heartbeat;
    std::function<manipulation::workflow::TaskOperationGateResult(
        const std::function<void()> &)> run_operation;
    std::function<void(const std::string &)> report_wait;
  };

  struct ActiveLease
  {
    std::string lease_id;
    std::string session_id;
    std::vector<std::string> resource_keys;
  };

  struct FullReadResult
  {
    std::optional<manipulation::workflow::PlanningSceneLease> lease;
    FailureKind failure{FailureKind::kInternal};
    std::string detail;

    [[nodiscard]] bool succeeded() const noexcept
    {
      return lease.has_value();
    }
  };

  struct ProofCheckResult
  {
    bool valid{false};
    FailureKind failure{FailureKind::kInternal};
    std::string detail;
  };

  struct MutationResult
  {
    std::optional<std::uint64_t> revision;
    FailureKind failure{FailureKind::kInternal};
    std::string detail;

    [[nodiscard]] bool succeeded() const noexcept
    {
      return revision.has_value();
    }
  };

  template<typename ServiceT>
  [[nodiscard]] ServiceCall<ServiceT> call_service(
    rclcpp::Client<ServiceT> & client,
    typename ServiceT::Request::SharedPtr request,
    const WaitControl & control,
    const std::string & label)
  {
    if (!request || !context_ || !control.wait_one_slice ||
      !control.heartbeat || !control.run_operation)
    {
      return {ServiceStatus::kFailed, {}, "invalid scene service call", false};
    }
    if (control.report_wait) {
      control.report_wait(label);
    }
    const auto ready = detail::wait_for_scene_condition(
      [&client]() {return client.service_is_ready();},
      control.wait_one_slice,
      [this]() {return context_ && context_->is_valid();});
    switch (ready) {
      case detail::InterruptibleWaitResult::kStopped:
        return {ServiceStatus::kStopped, {}, label + " wait stopped", false};
      case detail::InterruptibleWaitResult::kShutdown:
        return {ServiceStatus::kShutdown, {}, label + " owner shut down", false};
      case detail::InterruptibleWaitResult::kFailed:
        return {ServiceStatus::kFailed, {}, label + " discovery failed", false};
      case detail::InterruptibleWaitResult::kReady:
        break;
    }
    if (control.stopped && control.stopped()) {
      return {ServiceStatus::kStopped, {}, label + " stopped before send", false};
    }

    using Future = typename rclcpp::Client<ServiceT>::FutureAndRequestId;
    std::optional<Future> future;
    bool request_sent = false;
    bool shutdown_inside_gate = false;
    try {
      while (!future) {
        const auto gate = control.run_operation(
          [&]() {
            // Discovery readiness can race with ROS shutdown. Recheck inside
            // the lifecycle gate before crossing the transport boundary.
            if (!context_ || !context_->is_valid()) {
              shutdown_inside_gate = true;
              return;
            }
            // An exception from the transport cannot prove that the request did
            // not leave this process, so mutation callers must fail closed.
            request_sent = true;
            future.emplace(client.async_send_request(std::move(request)));
          });
        if (shutdown_inside_gate) {
          return {
            ServiceStatus::kShutdown, {},
            label + " owner shut down before send", false};
        }
        if (gate == manipulation::workflow::TaskOperationGateResult::kStopped) {
          return {ServiceStatus::kStopped, {}, label + " stopped before send", false};
        }
        if (gate == manipulation::workflow::TaskOperationGateResult::kPaused) {
          const auto paused = control.wait_one_slice();
          if (paused != detail::InterruptibleWaitResult::kReady) {
            return {
              paused == detail::InterruptibleWaitResult::kShutdown ?
              ServiceStatus::kShutdown : ServiceStatus::kStopped,
              {}, label + " stopped while dispatch was paused", false};
          }
        }
      }

      while (future->wait_for(kWaitSlice) != std::future_status::ready) {
        const auto response_wait = control.heartbeat();
        if (response_wait != detail::InterruptibleWaitResult::kReady) {
          client.remove_pending_request(*future);
          switch (response_wait) {
            case detail::InterruptibleWaitResult::kStopped:
              return {ServiceStatus::kStopped, {}, label + " response wait stopped", true};
            case detail::InterruptibleWaitResult::kShutdown:
              return {ServiceStatus::kShutdown, {}, label + " owner shut down", true};
            case detail::InterruptibleWaitResult::kFailed:
              return {ServiceStatus::kFailed, {}, label + " response wait failed", true};
            case detail::InterruptibleWaitResult::kReady:
              break;
          }
        }
      }
      auto response = future->get();
      if (!response) {
        return {ServiceStatus::kFailed, {}, label + " returned a null response", true};
      }
      return {ServiceStatus::kResponse, std::move(response), {}, true};
    } catch (const std::exception & error) {
      return {
        ServiceStatus::kFailed, {},
        label + " threw: " + error.what(), request_sent};
    } catch (...) {
      return {
        ServiceStatus::kFailed, {},
        label + " threw an unknown exception", request_sent};
    }
  }

  [[nodiscard]] WaitControl context_control(
    manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t generation) const;
  [[nodiscard]] WaitControl maintenance_control(
    manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t generation) const;
  [[nodiscard]] WaitControl planning_control(
    const PlanningStopToken & stop_token,
    std::uint64_t generation) const;

  [[nodiscard]] std::optional<ActiveLease> active_lease_for(
    const manipulation::workflow::PlanningSceneAccessProof & proof,
    std::string & detail) const;
  void set_active_lease(ActiveLease lease);
  void clear_active_lease(const std::string & lease_id) noexcept;

  [[nodiscard]] FullReadResult read_full_scene(
    const ActiveLease & active,
    const std::string & expected_session_id,
    std::uint64_t minimum_revision,
    manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t generation);
  [[nodiscard]] ProofCheckResult validate_exact_proof(
    const manipulation::workflow::PlanningSceneAccessProof & proof,
    const ActiveLease & active,
    manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t generation);
  [[nodiscard]] MutationResult apply_mutation(
    Apply::Request::SharedPtr request,
    manipulation::workflow::TransferExecutionContext & context,
    std::uint64_t generation);

  [[nodiscard]] moveit_msgs::msg::CollisionObject make_link_frame_object(
    const manipulation::workflow::PlanningSceneSnapshot & snapshot,
    const std::string & object_id,
    const std::string & attachment_link) const;

  [[nodiscard]] static FailureKind coordinator_failure(
    const manipulation_msgs::msg::PlanningSceneOperationResult & result);
  [[nodiscard]] static manipulation::task::TaskErrorCode task_error(FailureKind failure);
  [[nodiscard]] static CoordinatedSceneOutcome coordinated_outcome(FailureKind failure);

  void release_lease_id_best_effort(const std::string & lease_id) noexcept;
  [[nodiscard]] builtin_interfaces::msg::Duration requested_lease_ttl() const;

  const RosV2SceneAdapterConfig config_;
  const ::moveit::core::RobotModelConstPtr robot_model_;
  const manipulation::workflow::MeshResourceLoader & mesh_loader_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<Acquire>::SharedPtr acquire_client_;
  rclcpp::Client<Apply>::SharedPtr apply_client_;
  rclcpp::Client<GetScene>::SharedPtr get_scene_client_;
  rclcpp::Client<GetState>::SharedPtr state_client_;
  rclcpp::Client<Release>::SharedPtr release_client_;
  rclcpp::Client<Renew>::SharedPtr renew_client_;
  rclcpp::Client<Validate>::SharedPtr validate_client_;

  mutable std::mutex active_mutex_;
  std::optional<ActiveLease> active_lease_;
  std::mutex mutation_mutex_;
  mutable std::mutex operation_mutex_;
  detail::GoalStopGeneration stop_state_;
  std::atomic<std::uint64_t> operation_sequence_{0U};
};

}  // namespace manipulation::moveit

#endif  // ROS_V2_SCENE_ADAPTER_IMPL_HPP_
