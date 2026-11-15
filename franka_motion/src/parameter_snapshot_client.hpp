// Responsibility: Declares cancellable access to an authoritative node's parameter snapshot.

#ifndef PARAMETER_SNAPSHOT_CLIENT_HPP_
#define PARAMETER_SNAPSHOT_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"

namespace franka_motion::detail
{

/// Treat a true or throwing cooperative callback as a stop request.
inline bool cooperative_stop_requested(
  const std::function<bool()> & stop_requested) noexcept
{
  if (!stop_requested) {
    return false;
  }
  try {
    return stop_requested();
  } catch (...) {
    return true;
  }
}

/**
 * @brief Read a bounded parameter snapshot from one remote ROS node.
 *
 * The client owns the helper node and its private single-threaded executor for
 * its entire lifetime. Deadlines use steady time, so a simulation-clock jump
 * cannot extend startup indefinitely. This class is a private implementation
 * detail and is not thread-safe; one startup path must own and use it serially.
 */
class ParameterSnapshotClient final
{
public:
  using Clock = std::chrono::steady_clock;

  ParameterSnapshotClient(
    rclcpp::Node::SharedPtr helper,
    const std::string & remote_node_name,
    std::function<bool()> stop_requested = {});
  ~ParameterSnapshotClient();

  ParameterSnapshotClient(const ParameterSnapshotClient &) = delete;
  ParameterSnapshotClient & operator=(const ParameterSnapshotClient &) = delete;
  ParameterSnapshotClient(ParameterSnapshotClient &&) = delete;
  ParameterSnapshotClient & operator=(ParameterSnapshotClient &&) = delete;

  bool wait_for_service(Clock::duration timeout);

  std::optional<rclcpp::Parameter> wait_for_parameter(
    const std::string & name,
    Clock::time_point deadline);

  std::vector<std::string> wait_for_parameter_names(
    const std::string & prefix,
    std::uint64_t depth,
    Clock::time_point deadline);

  std::optional<std::vector<rclcpp::Parameter>> get_parameters(
    const std::vector<std::string> & names,
    Clock::time_point deadline);

  /**
   * @brief Retry one remote read when its first response is lost during startup.
   *
   * The first attempt receives only `initial_attempt_timeout`; a second attempt
   * may use the remainder of the original steady-clock deadline. A missing
   * second response still returns no snapshot, so callers can fail closed.
   */
  std::optional<std::vector<rclcpp::Parameter>> get_parameters_with_retry(
    const std::vector<std::string> & names,
    Clock::time_point deadline,
    Clock::duration initial_attempt_timeout);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace franka_motion::detail

#endif  // PARAMETER_SNAPSHOT_CLIENT_HPP_
