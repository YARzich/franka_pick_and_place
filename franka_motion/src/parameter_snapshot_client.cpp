// Responsibility: Performs bounded remote parameter reads and manifest polling for MoveIt
//   snapshots.

#include "parameter_snapshot_client.hpp"

#include <algorithm>
#include <exception>
#include <future>
#include <thread>
#include <utility>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/parameter_client.hpp"
#include "parameter_snapshot_helper_node.hpp"

namespace franka_motion::detail
{
namespace
{

constexpr auto kParameterPollPeriod = std::chrono::milliseconds(200);

ParameterSnapshotClient::Clock::duration remaining_until(
  const ParameterSnapshotClient::Clock::time_point deadline)
{
  using Clock = ParameterSnapshotClient::Clock;
  return std::max(Clock::duration::zero(), deadline - Clock::now());
}

ParameterSnapshotClient::Clock::duration next_poll_timeout(
  const ParameterSnapshotClient::Clock::time_point deadline)
{
  using Clock = ParameterSnapshotClient::Clock;
  const auto poll_period = std::chrono::duration_cast<Clock::duration>(kParameterPollPeriod);
  return std::min(remaining_until(deadline), poll_period);
}

}  // namespace

class ParameterSnapshotClient::Impl
{
public:
  Impl(
    rclcpp::Node::SharedPtr helper,
    const std::string & remote_node_name,
    std::function<bool()> stop_requested)
  : helper_(std::move(helper)),
    client_(std::make_shared<rclcpp::AsyncParametersClient>(helper_, remote_node_name)),
    executor_(make_parameter_snapshot_executor_options(
        helper_->get_node_base_interface()->get_context())),
    stop_requested_(std::move(stop_requested))
  {
    executor_.add_node(helper_);
  }

  ~Impl()
  {
    executor_.remove_node(helper_);
  }

  template<typename FutureT>
  bool wait_for_future(FutureT & future, const Clock::time_point deadline)
  {
    while (!should_stop() && Clock::now() < deadline) {
      const auto result = executor_.spin_until_future_complete(
        future, next_poll_timeout(deadline));
      if (result == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
      }
      if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
        return false;
      }
    }
    return false;
  }

  bool should_stop() const
  {
    return !rclcpp::ok(helper_->get_node_base_interface()->get_context()) ||
           cooperative_stop_requested(stop_requested_);
  }

  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::function<bool()> stop_requested_;
};

ParameterSnapshotClient::ParameterSnapshotClient(
  rclcpp::Node::SharedPtr helper,
  const std::string & remote_node_name,
  std::function<bool()> stop_requested)
: impl_(std::make_unique<Impl>(
      std::move(helper), remote_node_name, std::move(stop_requested)))
{
}

ParameterSnapshotClient::~ParameterSnapshotClient() = default;

bool ParameterSnapshotClient::wait_for_service(const Clock::duration timeout)
{
  const auto deadline = Clock::now() + timeout;
  while (!impl_->should_stop() && Clock::now() < deadline) {
    if (impl_->client_->wait_for_service(next_poll_timeout(deadline))) {
      return true;
    }
  }
  return false;
}

std::optional<std::vector<rclcpp::Parameter>> ParameterSnapshotClient::get_parameters(
  const std::vector<std::string> & names,
  const Clock::time_point deadline)
{
  auto future = impl_->client_->get_parameters(names);
  if (!impl_->wait_for_future(future, deadline)) {
    return std::nullopt;
  }
  return future.get();
}

std::optional<std::vector<rclcpp::Parameter>>
ParameterSnapshotClient::get_parameters_with_retry(
  const std::vector<std::string> & names,
  const Clock::time_point deadline,
  const Clock::duration initial_attempt_timeout)
{
  const auto initial_deadline = std::min(
    deadline, Clock::now() + std::max(Clock::duration::zero(), initial_attempt_timeout));
  try {
    if (const auto parameters = get_parameters(names, initial_deadline)) {
      return parameters;
    }
  } catch (const std::exception &) {
    // A source that is still bringing up can transiently reject its first request.
  }

  if (impl_->should_stop() || Clock::now() >= deadline) {
    return std::nullopt;
  }

  // AsyncParametersClient cannot remove a timed-out request through its public API.
  // This startup-only client therefore leaves at most one stale request, which is
  // released with the client after the complete snapshot copy.
  return get_parameters(names, deadline);
}

std::optional<rclcpp::Parameter> ParameterSnapshotClient::wait_for_parameter(
  const std::string & name,
  const Clock::time_point deadline)
{
  while (!impl_->should_stop() && Clock::now() < deadline) {
    try {
      const auto parameters = get_parameters({name}, deadline);
      if (parameters && !parameters->empty() &&
        parameters->front().get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        return parameters->front();
      }
    } catch (const std::exception &) {
      // Retry transient remote-service errors inside the original deadline.
    }

    const auto remaining = remaining_until(deadline);
    if (remaining > Clock::duration::zero()) {
      std::this_thread::sleep_for(std::min(remaining, next_poll_timeout(deadline)));
    }
  }
  return std::nullopt;
}

std::vector<std::string> ParameterSnapshotClient::wait_for_parameter_names(
  const std::string & prefix,
  const std::uint64_t depth,
  const Clock::time_point deadline)
{
  while (!impl_->should_stop() && Clock::now() < deadline) {
    try {
      auto future = impl_->client_->list_parameters({prefix}, depth);
      if (impl_->wait_for_future(future, deadline)) {
        const auto result = future.get();
        if (!result.names.empty()) {
          return result.names;
        }
      }
    } catch (const std::exception &) {
      // Retry transient remote-service errors inside the original deadline.
    }

    const auto remaining = remaining_until(deadline);
    if (remaining > Clock::duration::zero()) {
      std::this_thread::sleep_for(std::min(remaining, next_poll_timeout(deadline)));
    }
  }
  return {};
}

}  // namespace franka_motion::detail
