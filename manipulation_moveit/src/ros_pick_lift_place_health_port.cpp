// Responsibility: Waits for fresh typed health and every injected V2 backend readiness probe.

#include "manipulation_moveit/ros_pick_lift_place_health_port.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "manipulation_msgs/msg/system_health_snapshot.hpp"

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"

namespace manipulation::moveit
{
namespace
{

using Clock = std::chrono::steady_clock;
using HealthMessage = manipulation_msgs::msg::SystemHealthSnapshot;
using WaitingReason = manipulation::workflow::TransferWaitingReason;

// Private transport-safety invariants, deliberately absent from deployment YAML.
constexpr auto kHealthWaitSlice = std::chrono::milliseconds{10};
constexpr auto kHealthSnapshotMaxAge = std::chrono::seconds{2};

struct HealthState
{
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::optional<HealthMessage> latest;
  Clock::time_point received_at{};
  std::uint64_t update_count{0U};
};

struct ReadinessEvaluation
{
  PickLiftPlaceReadiness readiness;
  bool final_error{false};
};

std::string health_blocker(const HealthMessage & message)
{
  for (const auto & component : message.components) {
    if (component.required && component.level != component.LEVEL_OK) {
      return component.detail.empty() ?
             "required SystemHealth component is not ready" : component.detail;
    }
  }
  return "typed SystemHealth snapshot does not allow motion";
}

}  // namespace

void validateRosPickLiftPlaceHealthPortConfig(
  const RosPickLiftPlaceHealthPortConfig & config,
  const RosPickLiftPlaceReadinessProbes & probes)
{
  if (config.topic.empty()) {
    throw std::invalid_argument("pick/lift/place health topic must be non-empty");
  }
  if (!probes.planning || !probes.trajectory || !probes.gripper || !probes.scene) {
    throw std::invalid_argument(
            "pick/lift/place health requires planning, trajectory, gripper, and scene probes");
  }
}

class RosPickLiftPlaceHealthPort::Impl final
{
public:
  Impl(
    rclcpp::Node & node,
    RosPickLiftPlaceHealthPortConfig config,
    RosPickLiftPlaceReadinessProbes probes)
  : config_(std::move(config)), probes_(std::move(probes)),
    context_(node.get_node_base_interface()->get_context()),
    state_(std::make_shared<HealthState>()),
    callback_group_(node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
    validateRosPickLiftPlaceHealthPortConfig(config_, probes_);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto state = state_;
    subscription_ = node.create_subscription<HealthMessage>(
      config_.topic, qos,
      [state](HealthMessage::ConstSharedPtr message) {
        const std::lock_guard<std::mutex> lock(state->mutex);
        state->latest = *message;
        state->received_at = Clock::now();
        ++state->update_count;
        state->cv.notify_all();
      },
      options);
  }

  ~Impl()
  {
    request_stop();
  }

  void begin_goal() noexcept
  {
    goal_generation_.store(
      stop_generation_.load(std::memory_order_acquire), std::memory_order_release);
  }

  PickLiftPlaceReadiness cached_readiness() const
  {
    return evaluate().readiness;
  }

  PickLiftPlaceHealthResult wait_until_ready(
    manipulation::workflow::TransferExecutionContext & execution_context)
  {
    const auto generation = goal_generation_.load(std::memory_order_acquire);
    std::string last_reported_detail;
    for (;; ) {
      if (stopped(generation, execution_context)) {
        return PickLiftPlaceHealthResult::failure(
          PickLiftPlaceHealthOutcome::kCanceled,
          "pick/lift/place health wait was canceled");
      }
      if (!context_ || !context_->is_valid()) {
        return PickLiftPlaceHealthResult::failure(
          PickLiftPlaceHealthOutcome::kShutdown,
          "ROS context shut down during pick/lift/place health wait");
      }
      if (execution_context.checkpoint() ==
        manipulation::workflow::TransferExecutionControlResult::kStopped)
      {
        return PickLiftPlaceHealthResult::failure(
          PickLiftPlaceHealthOutcome::kCanceled,
          "pick/lift/place health wait stopped at a cooperative checkpoint");
      }

      const auto evaluation = evaluate();
      if (evaluation.final_error) {
        return PickLiftPlaceHealthResult::failure(
          PickLiftPlaceHealthOutcome::kInternalError,
          evaluation.readiness.detail);
      }
      if (evaluation.readiness.ready) {
        return PickLiftPlaceHealthResult::ready(evaluation.readiness.detail);
      }
      if (evaluation.readiness.detail != last_reported_detail) {
        manipulation::task::TaskStateSnapshot state;
        state.phase = manipulation::task::TaskPhase::kWaitingForResources;
        state.detail = evaluation.readiness.detail;
        execution_context.report_waiting(
          WaitingReason::kHealthReadiness, std::move(state));
        last_reported_detail = evaluation.readiness.detail;
      }

      std::unique_lock<std::mutex> lock(state_->mutex);
      const auto observed_update = state_->update_count;
      (void)state_->cv.wait_for(
        lock, kHealthWaitSlice,
        [this, generation, observed_update]() {
          return state_->update_count != observed_update ||
                 stop_generation_.load(std::memory_order_acquire) != generation;
        });
    }
  }

  void request_stop() noexcept
  {
    stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
    state_->cv.notify_all();
  }

private:
  ReadinessEvaluation evaluate() const
  {
    std::optional<HealthMessage> latest;
    Clock::time_point received_at;
    {
      const std::lock_guard<std::mutex> lock(state_->mutex);
      latest = state_->latest;
      received_at = state_->received_at;
    }
    if (!latest) {
      return {{false, "waiting for the first typed SystemHealth snapshot"}, false};
    }
    const auto now = Clock::now();
    if (now < received_at) {
      return {{false, "local steady health clock moved backwards"}, true};
    }
    if (now - received_at > kHealthSnapshotMaxAge) {
      return {{false, "typed SystemHealth snapshot is stale"}, false};
    }
    if (!latest->motion_ready) {
      return {{false, health_blocker(*latest)}, false};
    }

    const std::array<std::pair<const char *, const PickLiftPlaceReadinessProbe *>, 4>
    named_probes{{
      {"planning", &probes_.planning},
      {"trajectory", &probes_.trajectory},
      {"gripper", &probes_.gripper},
      {"scene", &probes_.scene}}};
    for (const auto & named_probe : named_probes) {
      try {
        const auto readiness = (*named_probe.second)();
        if (!readiness.ready) {
          return {{
            false,
            std::string{"waiting for "} + named_probe.first + ": " + readiness.detail},
            false};
        }
      } catch (const std::exception & error) {
        return {{
          false,
          std::string{named_probe.first} + " readiness probe threw: " + error.what()},
          true};
      } catch (...) {
        return {{
          false,
          std::string{named_probe.first} + " readiness probe threw an unknown exception"},
          true};
      }
    }
    return {{true, "typed health and all pick/lift/place dependencies are ready"}, false};
  }

  bool stopped(
    const std::uint64_t generation,
    const manipulation::workflow::TransferExecutionContext & execution_context) const noexcept
  {
    return execution_context.stop_requested() ||
           stop_generation_.load(std::memory_order_acquire) != generation;
  }

  const RosPickLiftPlaceHealthPortConfig config_;
  const RosPickLiftPlaceReadinessProbes probes_;
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<HealthState> state_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<HealthMessage>::SharedPtr subscription_;
  std::atomic<std::uint64_t> stop_generation_{0U};
  std::atomic<std::uint64_t> goal_generation_{0U};
};

RosPickLiftPlaceHealthPort::RosPickLiftPlaceHealthPort(
  rclcpp::Node & node,
  RosPickLiftPlaceHealthPortConfig config,
  RosPickLiftPlaceReadinessProbes probes)
: impl_(std::make_unique<Impl>(node, std::move(config), std::move(probes)))
{
}

RosPickLiftPlaceHealthPort::~RosPickLiftPlaceHealthPort() = default;

void RosPickLiftPlaceHealthPort::begin_goal() noexcept
{
  impl_->begin_goal();
}

PickLiftPlaceReadiness RosPickLiftPlaceHealthPort::cached_readiness() const
{
  return impl_->cached_readiness();
}

PickLiftPlaceHealthResult RosPickLiftPlaceHealthPort::wait_until_ready(
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->wait_until_ready(context);
}

void RosPickLiftPlaceHealthPort::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::moveit
