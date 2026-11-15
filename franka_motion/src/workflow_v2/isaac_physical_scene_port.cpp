// Responsibility: Implement the explicit Isaac physical/PlanningScene synchronization barrier.

#include "workflow_v2/isaac_physical_scene_port.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "manipulation/workflow/task_operation_gate.hpp"
#include "manipulation/workflow/task_stop_cause.hpp"

namespace franka_motion::workflow_v2
{
namespace
{

using manipulation::moveit::CoordinatedSceneOutcome;
using manipulation::moveit::CoordinatedSceneResult;
using manipulation::moveit::PickLiftPlaceReadiness;
using manipulation::workflow::TaskOperationGateResult;
using manipulation::workflow::TaskStopCause;
using manipulation::workflow::TransferExecutionContext;
using manipulation::workflow::TransferExecutionControlResult;
using SetBool = std_srvs::srv::SetBool;

constexpr auto kWaitSlice = std::chrono::milliseconds{50};

enum class PhysicalBarrierStatus : std::uint8_t
{
  kConfirmed,
  kCanceled,
  kShutdown,
  kStateUncertain,
  kInternalError,
};

struct PhysicalBarrierResult
{
  PhysicalBarrierStatus status{PhysicalBarrierStatus::kInternalError};
  std::string detail;

  [[nodiscard]] bool confirmed() const noexcept
  {
    return status == PhysicalBarrierStatus::kConfirmed;
  }
};

CoordinatedSceneOutcome stopped_outcome(const TransferExecutionContext & context) noexcept
{
  return context.stop_cause() == TaskStopCause::kShutdown ?
         CoordinatedSceneOutcome::kShutdown :
         CoordinatedSceneOutcome::kCanceled;
}

CoordinatedSceneResult barrier_failure(
  const PhysicalBarrierResult & result,
  const TransferExecutionContext & context)
{
  switch (result.status) {
    case PhysicalBarrierStatus::kConfirmed:
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kInternalError,
        "confirmed physical barrier entered failure mapping");
    case PhysicalBarrierStatus::kCanceled:
      return CoordinatedSceneResult::failure(stopped_outcome(context), result.detail);
    case PhysicalBarrierStatus::kShutdown:
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kShutdown, result.detail);
    case PhysicalBarrierStatus::kStateUncertain:
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain, result.detail);
    case PhysicalBarrierStatus::kInternalError:
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kInternalError, result.detail);
  }
  return CoordinatedSceneResult::failure(
    CoordinatedSceneOutcome::kInternalError,
    "unknown Isaac physical barrier outcome");
}

std::string operation_label(const bool attached)
{
  return attached ? "attach" : "detach";
}

const char * gate_result_label(const TaskOperationGateResult result) noexcept
{
  switch (result) {
    case TaskOperationGateResult::kExecuted:
      return "executed";
    case TaskOperationGateResult::kPaused:
      return "paused";
    case TaskOperationGateResult::kStopped:
      return "stopped";
  }
  return "unknown";
}

}  // namespace

void validateIsaacPhysicalScenePortConfig(
  const IsaacPhysicalScenePortConfig & config)
{
  if (config.object_id.empty()) {
    throw std::invalid_argument("Isaac physical attachment object_id must be non-empty");
  }
  if (config.service_name.empty()) {
    return;
  }
  if (config.service_name.front() != '/') {
    throw std::invalid_argument(
            "Isaac physical attachment service must be an absolute ROS name");
  }
  try {
    (void)rclcpp::expand_topic_or_service_name(
      config.service_name, "franka_transfer_object_sequence_server", "/", true);
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            std::string{"Isaac physical attachment service is invalid: "} + error.what());
  }
}

class IsaacPhysicalScenePort::Impl final
{
public:
  Impl(
    rclcpp::Node & node,
    manipulation::moveit::IPickLiftPlaceScenePort & scene,
    IsaacPhysicalScenePortConfig config,
    std::function<PickLiftPlaceReadiness()> scene_readiness)
  : scene_(scene),
    config_(std::move(config)),
    logger_(node.get_logger()),
    context_(node.get_node_base_interface()->get_context()),
    scene_readiness_(std::move(scene_readiness))
  {
    validateIsaacPhysicalScenePortConfig(config_);
    if (!scene_readiness_) {
      throw std::invalid_argument(
              "Isaac physical scene port requires raw scene readiness evidence");
    }
    if (!config_.service_name.empty()) {
      callback_group_ =
        node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      client_ = node.create_client<SetBool>(
        config_.service_name, rmw_qos_profile_services_default, callback_group_);
    }
  }

  ~Impl()
  {
    request_stop();
  }

  PickLiftPlaceReadiness cached_readiness() const
  {
    if (physical_state_unknown_.load(std::memory_order_acquire)) {
      return {
        false,
        "Isaac physical attachment state is uncertain; restart is required"};
    }
    const auto scene_readiness = scene_readiness_();
    if (!scene_readiness.ready || !client_) {
      return scene_readiness;
    }
    if (!context_ || !context_->is_valid()) {
      return {false, "ROS context for Isaac physical attachment is shut down"};
    }
    if (!client_->service_is_ready()) {
      return {
        false,
        "waiting for Isaac physical attachment service " + config_.service_name};
    }
    return {
      true,
      "scene coordinator and Isaac physical attachment service are ready"};
  }

  void begin_goal()
  {
    scene_.begin_goal();
    goal_generation_.store(
      stop_generation_.load(std::memory_order_acquire), std::memory_order_release);
  }

  CoordinatedSceneResult revalidate(
    const manipulation::moveit::StrategySceneRevalidationRequest & request,
    TransferExecutionContext & context)
  {
    return scene_.revalidate(request, context);
  }

  CoordinatedSceneResult attach(
    const manipulation::moveit::StrategySceneAttachRequest & request,
    TransferExecutionContext & context)
  {
    if (!client_) {
      return scene_.attach(request, context);
    }
    if (physical_state_unknown_.load(std::memory_order_acquire)) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        "Isaac physical attachment state is quarantined until restart");
    }
    if (request.object_id != config_.object_id) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kFailed,
        "Isaac physical attachment endpoint is scoped to object '" +
        config_.object_id + "', not '" + request.object_id + "'");
    }
    const auto physical = set_attached(true, context);
    if (!physical.confirmed()) {
      quarantine_if_uncertain(physical);
      return barrier_failure(physical, context);
    }
    log_confirmed_ack(request.object_id, "attach", physical.detail);
    // The two representations intentionally differ inside this narrow window.
    // Quarantine first, and clear it only after the coordinator proves success.
    physical_state_unknown_.store(true, std::memory_order_release);
    try {
      auto coordinated = scene_.attach(request, context);
      if (!coordinated.succeeded()) {
        return CoordinatedSceneResult::failure(
          CoordinatedSceneOutcome::kStateUncertain,
          "Isaac physical attach was confirmed but PlanningScene attach failed: " +
          coordinated.detail());
      }
      physical_state_unknown_.store(false, std::memory_order_release);
      return coordinated;
    } catch (const std::exception & error) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        std::string{"Isaac physical attach was confirmed but PlanningScene attach threw: "} +
        error.what());
    } catch (...) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        "Isaac physical attach was confirmed but PlanningScene attach threw");
    }
  }

  CoordinatedSceneResult detach(
    const manipulation::moveit::StrategySceneDetachRequest & request,
    TransferExecutionContext & context)
  {
    if (!client_) {
      return scene_.detach(request, context);
    }
    if (physical_state_unknown_.load(std::memory_order_acquire)) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        "Isaac physical attachment state is quarantined until restart");
    }
    if (request.object_id != config_.object_id) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kFailed,
        "Isaac physical attachment endpoint is scoped to object '" +
        config_.object_id + "', not '" + request.object_id + "'");
    }
    const auto physical = set_attached(false, context);
    if (!physical.confirmed()) {
      quarantine_if_uncertain(physical);
      return barrier_failure(physical, context);
    }
    log_confirmed_ack(request.object_id, "detach", physical.detail);
    physical_state_unknown_.store(true, std::memory_order_release);
    try {
      auto coordinated = scene_.detach(request, context);
      if (!coordinated.succeeded()) {
        return CoordinatedSceneResult::failure(
          CoordinatedSceneOutcome::kStateUncertain,
          "Isaac physical detach was confirmed but PlanningScene detach failed: " +
          coordinated.detail());
      }
      physical_state_unknown_.store(false, std::memory_order_release);
      return coordinated;
    } catch (const std::exception & error) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        std::string{"Isaac physical detach was confirmed but PlanningScene detach threw: "} +
        error.what());
    } catch (...) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kStateUncertain,
        "Isaac physical detach was confirmed but PlanningScene detach threw");
    }
  }

  void request_stop() noexcept
  {
    stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
    scene_.request_stop();
  }

private:
  void log_confirmed_ack(
    const std::string & object_id,
    const char * operation,
    const std::string & detail) const
  {
    RCLCPP_INFO(
      logger_,
      "Isaac physical attachment ACK confirmed: object=%s operation=%s detail=%s",
      object_id.c_str(), operation, detail.c_str());
  }

  void quarantine_if_uncertain(const PhysicalBarrierResult & result) noexcept
  {
    if (result.status == PhysicalBarrierStatus::kStateUncertain) {
      physical_state_unknown_.store(true, std::memory_order_release);
    }
  }

  [[nodiscard]] bool stopped(
    const std::uint64_t generation,
    const TransferExecutionContext & context) const noexcept
  {
    return context.stop_requested() ||
           stop_generation_.load(std::memory_order_acquire) != generation;
  }

  [[nodiscard]] PhysicalBarrierResult set_attached(
    const bool attached,
    TransferExecutionContext & execution_context)
  {
    if (!client_ || !context_) {
      return {
        PhysicalBarrierStatus::kInternalError,
        "Isaac physical attachment client is not configured"};
    }
    const auto generation = goal_generation_.load(std::memory_order_acquire);
    const auto label = operation_label(attached);
    std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_);

    while (!client_->service_is_ready()) {
      if (!context_->is_valid()) {
        return {
          PhysicalBarrierStatus::kShutdown,
          "ROS context shut down before Isaac physical " + label};
      }
      if (stopped(generation, execution_context) ||
        execution_context.checkpoint() == TransferExecutionControlResult::kStopped)
      {
        return {
          PhysicalBarrierStatus::kCanceled,
          "Isaac physical " + label + " stopped before service dispatch"};
      }
      (void)client_->wait_for_service(kWaitSlice);
    }

    using Future = rclcpp::Client<SetBool>::FutureAndRequestId;
    std::optional<Future> future;
    bool request_may_have_been_sent = false;
    bool shutdown_inside_gate = false;
    try {
      RCLCPP_INFO(
        logger_,
        "Isaac physical dispatch entering operation gate: "
        "object=%s operation=%s service=%s",
        config_.object_id.c_str(), label.c_str(), config_.service_name.c_str());
      while (!future) {
        const auto gate = execution_context.run_operation(
          [&]() {
            if (!context_->is_valid()) {
              shutdown_inside_gate = true;
              return;
            }
            // run_operation() owns the non-recursive lifecycle session mutex.
            // Querying execution_context here would re-enter that mutex through
            // stop_requested()/stop_cause(). The gate itself has already
            // linearized lifecycle stop; only the port-local generation can
            // change independently while this callback runs.
            if (stop_generation_.load(std::memory_order_acquire) != generation) {
              return;
            }
            auto request = std::make_shared<SetBool::Request>();
            request->data = attached;
            RCLCPP_INFO(
              logger_,
              "Isaac physical dispatch before async_send_request: "
              "object=%s operation=%s service=%s",
              config_.object_id.c_str(), label.c_str(), config_.service_name.c_str());
            request_may_have_been_sent = true;
            future.emplace(client_->async_send_request(std::move(request)));
            RCLCPP_INFO(
              logger_,
              "Isaac physical dispatch after async_send_request: "
              "object=%s operation=%s service=%s",
              config_.object_id.c_str(), label.c_str(), config_.service_name.c_str());
          });
        if (gate != TaskOperationGateResult::kPaused) {
          RCLCPP_INFO(
            logger_,
            "Isaac physical dispatch operation gate result: "
            "object=%s operation=%s result=%s request_dispatched=%s",
            config_.object_id.c_str(), label.c_str(), gate_result_label(gate),
            request_may_have_been_sent ? "true" : "false");
        }
        if (shutdown_inside_gate) {
          return {
            PhysicalBarrierStatus::kShutdown,
            "ROS context shut down before Isaac physical " + label + " dispatch"};
        }
        if (gate == TaskOperationGateResult::kStopped ||
          stopped(generation, execution_context))
        {
          if (request_may_have_been_sent) {
            if (future) {
              (void)client_->remove_pending_request(*future);
            }
            return {
              PhysicalBarrierStatus::kStateUncertain,
              "Isaac physical " + label +
              " request crossed dispatch while the goal stopped; outcome is unknown"};
          }
          return {
            PhysicalBarrierStatus::kCanceled,
            "Isaac physical " + label + " stopped before service dispatch"};
        }
        if (gate == TaskOperationGateResult::kPaused) {
          if (execution_context.checkpoint() ==
            TransferExecutionControlResult::kStopped)
          {
            return {
              PhysicalBarrierStatus::kCanceled,
              "Isaac physical " + label + " stopped while dispatch was paused"};
          }
        }
      }

      while (future->wait_for(kWaitSlice) != std::future_status::ready) {
        if (!context_->is_valid() ||
          stopped(generation, execution_context) ||
          execution_context.heartbeat() == TransferExecutionControlResult::kStopped)
        {
          (void)client_->remove_pending_request(*future);
          return {
            PhysicalBarrierStatus::kStateUncertain,
            "Isaac physical " + label +
            " request was dispatched but its terminal outcome is unknown"};
        }
      }
      const auto response = future->get();
      if (!response) {
        RCLCPP_INFO(
          logger_,
          "Isaac physical terminal response: "
          "object=%s operation=%s received=false",
          config_.object_id.c_str(), label.c_str());
        return {
          PhysicalBarrierStatus::kStateUncertain,
          "Isaac physical " + label + " returned a null response after dispatch"};
      }
      RCLCPP_INFO(
        logger_,
        "Isaac physical terminal response: "
        "object=%s operation=%s received=true success=%s detail=%s",
        config_.object_id.c_str(), label.c_str(),
        response->success ? "true" : "false", response->message.c_str());
      if (!response->success) {
        return {
          PhysicalBarrierStatus::kStateUncertain,
          response->message.empty() ?
          "Isaac physical " + label +
          " returned failure after dispatch; state is unknown" :
          "Isaac physical " + label +
          " returned failure after dispatch; state is unknown: " + response->message};
      }
      return {
        PhysicalBarrierStatus::kConfirmed,
        response->message.empty() ?
        "Isaac physical " + label + " confirmed" : response->message};
    } catch (const std::exception & error) {
      return {
        request_may_have_been_sent ?
        PhysicalBarrierStatus::kStateUncertain :
        PhysicalBarrierStatus::kInternalError,
        "Isaac physical " + label + " call failed: " + error.what()};
    } catch (...) {
      return {
        request_may_have_been_sent ?
        PhysicalBarrierStatus::kStateUncertain :
        PhysicalBarrierStatus::kInternalError,
        "Isaac physical " + label + " call failed with an unknown error"};
    }
  }

  manipulation::moveit::IPickLiftPlaceScenePort & scene_;
  IsaacPhysicalScenePortConfig config_;
  rclcpp::Logger logger_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<SetBool>::SharedPtr client_;
  std::function<PickLiftPlaceReadiness()> scene_readiness_;
  std::mutex dispatch_mutex_;
  std::atomic<std::uint64_t> stop_generation_{0U};
  std::atomic<std::uint64_t> goal_generation_{0U};
  std::atomic_bool physical_state_unknown_{false};
};

IsaacPhysicalScenePort::IsaacPhysicalScenePort(
  rclcpp::Node & node,
  manipulation::moveit::IPickLiftPlaceScenePort & scene,
  IsaacPhysicalScenePortConfig config,
  std::function<PickLiftPlaceReadiness()> scene_readiness)
: impl_(std::make_unique<Impl>(
      node, scene, std::move(config), std::move(scene_readiness)))
{
}

IsaacPhysicalScenePort::~IsaacPhysicalScenePort() = default;

PickLiftPlaceReadiness IsaacPhysicalScenePort::cached_readiness() const
{
  return impl_->cached_readiness();
}

void IsaacPhysicalScenePort::begin_goal()
{
  impl_->begin_goal();
}

CoordinatedSceneResult IsaacPhysicalScenePort::revalidate(
  const manipulation::moveit::StrategySceneRevalidationRequest & request,
  TransferExecutionContext & context)
{
  return impl_->revalidate(request, context);
}

CoordinatedSceneResult IsaacPhysicalScenePort::attach(
  const manipulation::moveit::StrategySceneAttachRequest & request,
  TransferExecutionContext & context)
{
  return impl_->attach(request, context);
}

CoordinatedSceneResult IsaacPhysicalScenePort::detach(
  const manipulation::moveit::StrategySceneDetachRequest & request,
  TransferExecutionContext & context)
{
  return impl_->detach(request, context);
}

void IsaacPhysicalScenePort::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace franka_motion::workflow_v2
