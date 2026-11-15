// Responsibility: Maps V2 commands to quiescence-gated, acceptance-timed ROS actions.

#include "manipulation_moveit/ros_v2_physical_ports.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "accepted_physical_action.hpp"
#include "quiescence_policy.hpp"

namespace manipulation::moveit
{
namespace
{

using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using MoveItErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
using ResultCode = rclcpp_action::ResultCode;

using detail::JointTarget;
using detail::RequiredJointSample;
using detail::QuiescenceClock;
using detail::QuiescenceEvidence;
using detail::QuiescenceTracker;

constexpr std::size_t kJointStateQueueCapacity = 256U;
constexpr auto kQuiescenceWaitSlice = std::chrono::milliseconds{5};

struct TrajectoryJointTargets
{
  JointTarget start;
  JointTarget terminal;
};

std::optional<TrajectoryJointTargets> trajectory_joint_targets(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & error)
{
  error.clear();
  if (!trajectory.multi_dof_joint_trajectory.joint_names.empty() ||
    !trajectory.multi_dof_joint_trajectory.points.empty())
  {
    error =
      "ExecuteTrajectory cannot prove quiescence for a multi-DOF trajectory from JointState";
    return std::nullopt;
  }
  const auto & joint_trajectory = trajectory.joint_trajectory;
  if (joint_trajectory.points.empty() || joint_trajectory.joint_names.empty()) {
    error = "ExecuteTrajectory requires a joint trajectory with named waypoints";
    return std::nullopt;
  }
  const auto & start = joint_trajectory.points.front().positions;
  const auto & terminal = joint_trajectory.points.back().positions;
  const auto & terminal_velocities = joint_trajectory.points.back().velocities;
  if (start.size() != joint_trajectory.joint_names.size() ||
    terminal.size() != joint_trajectory.joint_names.size())
  {
    error = "ExecuteTrajectory start and terminal waypoints must cover every named joint";
    return std::nullopt;
  }
  if (!terminal_velocities.empty() &&
    terminal_velocities.size() != joint_trajectory.joint_names.size())
  {
    error =
      "ExecuteTrajectory terminal waypoint velocities must cover every named joint";
    return std::nullopt;
  }

  std::unordered_set<std::string> unique_names;
  unique_names.reserve(joint_trajectory.joint_names.size());
  for (std::size_t index = 0U; index < joint_trajectory.joint_names.size(); ++index) {
    if (joint_trajectory.joint_names[index].empty() ||
      !unique_names.insert(joint_trajectory.joint_names[index]).second ||
      !std::isfinite(start[index]) || !std::isfinite(terminal[index]) ||
      (!terminal_velocities.empty() && !std::isfinite(terminal_velocities[index])))
    {
      error = "ExecuteTrajectory contains malformed joint waypoint evidence";
      return std::nullopt;
    }
  }
  return TrajectoryJointTargets{
    JointTarget{joint_trajectory.joint_names, start, {}},
    JointTarget{joint_trajectory.joint_names, terminal, terminal_velocities}};
}

std::int64_t stamp_nanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::optional<RequiredJointSample> required_joint_sample(
  const sensor_msgs::msg::JointState & sample,
  const JointTarget & target,
  std::string & error)
{
  error.clear();
  if (sample.name.empty() || sample.name.size() != sample.position.size()) {
    error = "JointState has no complete name/position arrays";
    return std::nullopt;
  }
  if (!sample.velocity.empty() && sample.velocity.size() != sample.name.size()) {
    error = "JointState velocity array does not match its names";
    return std::nullopt;
  }

  std::unordered_map<std::string, std::size_t> indices;
  indices.reserve(sample.name.size());
  for (std::size_t index = 0U; index < sample.name.size(); ++index) {
    if (sample.name[index].empty() ||
      !indices.emplace(sample.name[index], index).second)
    {
      error = "JointState contains an empty or duplicate joint name";
      return std::nullopt;
    }
  }

  RequiredJointSample result;
  result.positions.reserve(target.names.size());
  if (!sample.velocity.empty()) {
    result.velocities.emplace();
    result.velocities->reserve(target.names.size());
  }
  for (const auto & name : target.names) {
    const auto found = indices.find(name);
    if (found == indices.cend()) {
      error = "JointState does not contain required trajectory joint '" + name + "'";
      return std::nullopt;
    }
    const auto index = found->second;
    if (!std::isfinite(sample.position[index])) {
      error = "JointState contains a non-finite required joint position";
      return std::nullopt;
    }
    result.positions.push_back(sample.position[index]);
    if (result.velocities) {
      if (!std::isfinite(sample.velocity[index])) {
        error = "JointState contains a non-finite required joint velocity";
        return std::nullopt;
      }
      result.velocities->push_back(sample.velocity[index]);
    }
  }
  result.stamp_nanoseconds = stamp_nanoseconds(sample.header.stamp);
  if (sample.header.stamp.nanosec >= 1000000000U ||
    result.stamp_nanoseconds <= 0)
  {
    error = "JointState requires a normalized non-zero timestamp; stamp_ns=" +
      std::to_string(result.stamp_nanoseconds);
    return std::nullopt;
  }
  return result;
}

template<typename ActionT>
PhysicalCommandResult physical_transport_failure(
  const detail::AcceptedPhysicalActionResult<ActionT> & call)
{
  PhysicalCommandOutcome outcome;
  switch (call.status) {
    case detail::AcceptedPhysicalActionStatus::kResultReceived:
      outcome = PhysicalCommandOutcome::kInternalError;
      break;
    case detail::AcceptedPhysicalActionStatus::kRejected:
      outcome = PhysicalCommandOutcome::kRejected;
      break;
    case detail::AcceptedPhysicalActionStatus::kCanceled:
      outcome = PhysicalCommandOutcome::kCanceled;
      break;
    case detail::AcceptedPhysicalActionStatus::kTimedOut:
      outcome = PhysicalCommandOutcome::kTimedOut;
      break;
    case detail::AcceptedPhysicalActionStatus::kUnavailable:
      outcome = PhysicalCommandOutcome::kUnavailable;
      break;
    case detail::AcceptedPhysicalActionStatus::kShutdown:
      outcome = PhysicalCommandOutcome::kShutdown;
      break;
    case detail::AcceptedPhysicalActionStatus::kInternalError:
      outcome = PhysicalCommandOutcome::kInternalError;
      break;
  }
  return PhysicalCommandResult::failure(
    outcome, detail::physical_action_safety(call), call.detail);
}

PhysicalCommandResult execute_trajectory_result(
  const detail::AcceptedPhysicalActionResult<ExecuteTrajectory> & call)
{
  if (call.status != detail::AcceptedPhysicalActionStatus::kResultReceived) {
    return physical_transport_failure(call);
  }
  const auto & wrapped = *call.result;
  if (!wrapped.result) {
    return PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kInternalError,
      PhysicalCommandSafety::kTerminalSafe,
      "ExecuteTrajectory returned a terminal response without a result payload");
  }
  const auto moveit_code = wrapped.result->error_code.val;
  if (wrapped.code == ResultCode::CANCELED || moveit_code == MoveItErrorCodes::PREEMPTED) {
    return PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kCanceled,
      PhysicalCommandSafety::kTerminalSafe,
      "ExecuteTrajectory reached a canceled terminal state");
  }
  if (moveit_code == MoveItErrorCodes::TIMED_OUT) {
    return PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kTimedOut,
      PhysicalCommandSafety::kTerminalSafe,
      "ExecuteTrajectory reported a terminal controller timeout");
  }
  if (wrapped.code != ResultCode::SUCCEEDED || moveit_code != MoveItErrorCodes::SUCCESS) {
    std::string detail_text = "ExecuteTrajectory failed with MoveIt code " +
      std::to_string(moveit_code);
    if (!wrapped.result->error_code.message.empty()) {
      detail_text += ": " + wrapped.result->error_code.message;
    }
    return PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kFailed,
      PhysicalCommandSafety::kTerminalSafe,
      std::move(detail_text));
  }
  return PhysicalCommandResult::success("ExecuteTrajectory completed successfully");
}

template<typename ActionT>
GripperPostcondition unresolved_gripper_postcondition(
  const detail::AcceptedPhysicalActionResult<ActionT> & call) noexcept
{
  return detail::physical_action_safety(call) == PhysicalCommandSafety::kNotAccepted ?
         GripperPostcondition::kNone : GripperPostcondition::kUnknown;
}

StrategyGripperCommandResult gripper_transport_failure(
  const detail::AcceptedPhysicalActionResult<GripperCommand> & call)
{
  const auto physical = physical_transport_failure(call);
  return StrategyGripperCommandResult::failure(
    physical.outcome(), physical.safety(), unresolved_gripper_postcondition(call),
    physical.detail());
}

StrategyGripperCommandResult gripper_terminal_result(
  const detail::AcceptedPhysicalActionResult<GripperCommand> & call,
  const StrategyGripperCommandRequest & request)
{
  if (call.status != detail::AcceptedPhysicalActionStatus::kResultReceived) {
    return gripper_transport_failure(call);
  }
  const auto & wrapped = *call.result;
  if (!wrapped.result) {
    return StrategyGripperCommandResult::failure(
      PhysicalCommandOutcome::kInternalError,
      PhysicalCommandSafety::kTerminalSafe,
      GripperPostcondition::kUnknown,
      "GripperCommand returned a terminal response without a result payload");
  }
  if (wrapped.code == ResultCode::CANCELED) {
    return StrategyGripperCommandResult::failure(
      PhysicalCommandOutcome::kCanceled,
      PhysicalCommandSafety::kTerminalSafe,
      GripperPostcondition::kUnknown,
      "GripperCommand reached a canceled terminal state");
  }
  if (!std::isfinite(wrapped.result->position) || wrapped.result->position < 0.0 ||
    !std::isfinite(wrapped.result->effort))
  {
    return StrategyGripperCommandResult::failure(
      PhysicalCommandOutcome::kInternalError,
      PhysicalCommandSafety::kTerminalSafe,
      GripperPostcondition::kUnknown,
      "GripperCommand returned invalid position or effort evidence");
  }
  if (wrapped.code != ResultCode::SUCCEEDED) {
    return StrategyGripperCommandResult::failure(
      PhysicalCommandOutcome::kFailed,
      PhysicalCommandSafety::kTerminalSafe,
      GripperPostcondition::kUnknown,
      "GripperCommand action did not succeed");
  }

  const auto confirmed = request.kind == GripperCommandKind::kOpen ?
    GripperPostcondition::kOpenConfirmed : GripperPostcondition::kClosedConfirmed;
  if (wrapped.result->reached_goal) {
    return StrategyGripperCommandResult::success(
      confirmed, "GripperCommand reached the requested target");
  }
  if (request.kind == GripperCommandKind::kClose && request.allow_stall &&
    wrapped.result->stalled)
  {
    return StrategyGripperCommandResult::success(
      GripperPostcondition::kClosedConfirmed,
      "GripperCommand stall was accepted as grasp contact");
  }
  return StrategyGripperCommandResult::failure(
    PhysicalCommandOutcome::kFailed,
    PhysicalCommandSafety::kTerminalSafe,
    GripperPostcondition::kUnknown,
    wrapped.result->stalled ?
    "GripperCommand stalled without an allowed close-contact policy" :
    "GripperCommand completed without reaching its target");
}

enum class QuiescenceWaitStatus
{
  kReady,
  kTimedOut,
  kCanceled,
  kShutdown,
  kInvalid,
};

struct QuiescenceWaitResult
{
  QuiescenceWaitStatus status{QuiescenceWaitStatus::kInvalid};
  std::string detail{"joint-state quiescence result was not initialized"};
};

struct ReceivedJointState
{
  std::uint64_t sequence{0U};
  sensor_msgs::msg::JointState message;
  std::chrono::steady_clock::time_point received_at;
};

class ScopedConvergenceActivity final
{
public:
  explicit ScopedConvergenceActivity(std::atomic_bool & active) noexcept
  : active_(active)
  {
    active_.store(true, std::memory_order_release);
  }

  ~ScopedConvergenceActivity()
  {
    active_.store(false, std::memory_order_release);
  }

  ScopedConvergenceActivity(const ScopedConvergenceActivity &) = delete;
  ScopedConvergenceActivity & operator=(const ScopedConvergenceActivity &) = delete;

private:
  std::atomic_bool & active_;
};

}  // namespace

void validateRosTrajectoryExecutionPortConfig(
  const RosTrajectoryExecutionPortConfig & config)
{
  if (config.action_name.empty() || config.joint_state_topic.empty()) {
    throw std::invalid_argument(
            "ExecuteTrajectory action and JointState topic names must be non-empty");
  }
}

void validateRosStrategyGripperPortConfig(const RosStrategyGripperPortConfig & config)
{
  if (config.action_name.empty()) {
    throw std::invalid_argument("GripperCommand action name must be non-empty");
  }
}

class RosTrajectoryExecutionPort::Impl final
{
public:
  Impl(rclcpp::Node & node, RosTrajectoryExecutionPortConfig config)
  : config_(std::move(config)),
    context_(node.get_node_base_interface()->get_context()),
    clock_(node.get_clock()),
    callback_group_(node.create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    validateRosTrajectoryExecutionPortConfig(config_);
    client_ = rclcpp_action::create_client<ExecuteTrajectory>(
      &node, config_.action_name, callback_group_);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    joint_state_subscription_ = node.create_subscription<sensor_msgs::msg::JointState>(
      config_.joint_state_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        receive_joint_state(*message);
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
    if (convergence_active_.load(std::memory_order_acquire)) {
      return {false, "ExecuteTrajectory joint-state convergence is still active"};
    }
    if (terminal_state_unknown_.load(std::memory_order_acquire)) {
      return {false, "previous ExecuteTrajectory terminal physical state is unknown"};
    }
    if (active_slot_.dispatch_unknown()) {
      return {false, "previous ExecuteTrajectory dispatch outcome is unknown"};
    }
    if (active_slot_.busy()) {
      return {false, "previous ExecuteTrajectory goal is not terminal"};
    }
    if (!client_->action_server_is_ready()) {
      return {false, "ExecuteTrajectory action is unavailable"};
    }
    return {true, "ExecuteTrajectory action is ready"};
  }

  PhysicalCommandResult execute(
    const TrajectoryExecutionRequest & request,
    const std::chrono::steady_clock::duration accepted_motion_timeout,
    manipulation::workflow::TransferExecutionContext & execution_context)
  {
    std::string target_error;
    auto targets = trajectory_joint_targets(request.trajectory, target_error);
    if (!targets) {
      return PhysicalCommandResult::failure(
        PhysicalCommandOutcome::kRejected,
        PhysicalCommandSafety::kNotAccepted,
        std::move(target_error));
    }
    if (accepted_motion_timeout <= std::chrono::steady_clock::duration::zero()) {
      return PhysicalCommandResult::failure(
        PhysicalCommandOutcome::kRejected,
        PhysicalCommandSafety::kNotAccepted,
        "accepted trajectory motion timeout must be positive");
    }

    ExecuteTrajectory::Goal goal;
    goal.trajectory = request.trajectory;
    const auto stop_requested = stop_predicate();
    const auto call = dispatch_after_quiescence(
      goal, targets->start, accepted_motion_timeout,
      execution_context, stop_requested);
    auto action_result = execute_trajectory_result(call);
    if (!action_result.succeeded()) {
      return action_result;
    }

    std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_);
    const auto terminal_evidence = wait_for_quiescence(
      targets->terminal, execution_context, stop_requested,
      "after successful ExecuteTrajectory result", call.accepted_deadline);
    dispatch_lock.unlock();
    if (terminal_evidence.status != QuiescenceWaitStatus::kReady) {
      terminal_state_unknown_.store(true, std::memory_order_release);
      return quiescence_failure(
        terminal_evidence, PhysicalCommandSafety::kActiveStateUnknown);
    }
    return PhysicalCommandResult::success(
      "ExecuteTrajectory completed and fresh JointState samples proved terminal quiescence");
  }

  void request_stop() noexcept
  {
    // Publish stop intent before waiting for the dispatch gate: an unbounded
    // quiescence wait owns that gate and must be able to wake itself. Acquiring
    // the gate below then linearizes stop with send: either the waiter observes
    // this generation before send, or the exact installed goal is canceled.
    stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
    joint_state_cv_.notify_all();
    const std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
    active_slot_.request_cancel();
  }

private:
  using AcceptedResult = detail::AcceptedPhysicalActionResult<ExecuteTrajectory>;
  using AcceptedState = detail::AcceptedPhysicalActionState<ExecuteTrajectory>;

  void receive_joint_state(const sensor_msgs::msg::JointState & message)
  {
    const std::lock_guard<std::mutex> lock(joint_state_mutex_);
    const auto sequence = ++joint_state_sequence_;
    joint_state_queue_.push_back(
      ReceivedJointState{sequence, message, std::chrono::steady_clock::now()});
    while (joint_state_queue_.size() > kJointStateQueueCapacity) {
      joint_state_queue_.pop_front();
    }
    joint_state_cv_.notify_all();
  }

  QuiescenceWaitResult wait_for_quiescence(
    const JointTarget & target,
    manipulation::workflow::TransferExecutionContext & execution_context,
    const std::function<bool()> & stop_requested,
    const std::string & phase,
    const std::optional<std::chrono::steady_clock::time_point> deadline = std::nullopt)
  {
    const ScopedConvergenceActivity activity(convergence_active_);
    QuiescenceTracker tracker;
    if (!context_ || !context_->is_valid()) {
      return {
        QuiescenceWaitStatus::kShutdown,
        "ROS context shut down before joint-state quiescence " + phase};
    }

    std::uint64_t last_sequence{0U};
    std::int64_t barrier_stamp_nanoseconds{0};
    try {
      const std::lock_guard<std::mutex> lock(joint_state_mutex_);
      last_sequence = joint_state_sequence_;
      barrier_stamp_nanoseconds = clock_->now().nanoseconds();
      if (barrier_stamp_nanoseconds <= 0) {
        return {
          QuiescenceWaitStatus::kInvalid,
          "joint-state freshness requires a non-zero ROS clock " + phase +
          "; now_ns=" + std::to_string(barrier_stamp_nanoseconds)};
      }
    } catch (const std::exception & error) {
      return {
        QuiescenceWaitStatus::kInvalid,
        "failed to establish joint-state freshness barrier " + phase + ": " +
        error.what()};
    } catch (...) {
      return {
        QuiescenceWaitStatus::kInvalid,
        "unknown failure establishing joint-state freshness barrier " + phase};
    }

    QuiescenceClock clock_evidence{barrier_stamp_nanoseconds};
    std::string clock_error;
    const auto current_ros_time = [&]() -> std::optional<std::int64_t> {
        try {
          const auto ros_now = clock_->now().nanoseconds();
          if (!clock_evidence.observe(ros_now, clock_error)) {
            clock_error += "; phase=" + phase;
            return std::nullopt;
          }
          return ros_now;
        } catch (const std::exception & error) {
          clock_error = "cannot read ROS clock during joint-state quiescence " +
            phase + ": " + error.what();
          return std::nullopt;
        }
      };

    for (;; ) {
      if (!context_ || !context_->is_valid()) {
        return {
          QuiescenceWaitStatus::kShutdown,
          "ROS context shut down while waiting for joint-state quiescence " + phase};
      }
      if (stop_requested() || execution_context.stop_requested()) {
        return {
          QuiescenceWaitStatus::kCanceled,
          "trajectory execution stopped while waiting for joint-state quiescence " + phase};
      }
      if (!current_ros_time()) {
        return {QuiescenceWaitStatus::kInvalid, clock_error};
      }
      const auto now = std::chrono::steady_clock::now();
      if (deadline && now >= *deadline) {
        return {
          QuiescenceWaitStatus::kTimedOut,
          "accepted trajectory motion budget expired while waiting for joint-state "
          "quiescence " + phase + "; last evidence: " + tracker.diagnostic(target)};
      }

      std::vector<ReceivedJointState> samples;
      {
        std::unique_lock<std::mutex> lock(joint_state_mutex_);
        const auto wait_deadline = deadline ?
          std::min(*deadline, now + kQuiescenceWaitSlice) :
          now + kQuiescenceWaitSlice;
        joint_state_cv_.wait_until(
          lock, wait_deadline,
          [this, last_sequence, &stop_requested]() {
            return joint_state_sequence_ > last_sequence ||
                   stop_requested() || !context_ || !context_->is_valid();
          });
        for (const auto & sample : joint_state_queue_) {
          if (sample.sequence > last_sequence) {
            samples.push_back(sample);
          }
        }
        if (!samples.empty()) {
          last_sequence = samples.back().sequence;
          while (!joint_state_queue_.empty() &&
            joint_state_queue_.front().sequence <= last_sequence)
          {
            joint_state_queue_.pop_front();
          }
        }
      }

      for (const auto & sample : samples) {
        const auto ros_now = current_ros_time();
        if (!ros_now) {
          return {QuiescenceWaitStatus::kInvalid, clock_error};
        }
        std::string evidence_error;
        const auto required_sample = required_joint_sample(sample.message, target, evidence_error);
        const auto evidence = required_sample ? tracker.observe(
          *required_sample, target, barrier_stamp_nanoseconds, *ros_now,
          sample.received_at, evidence_error) : QuiescenceEvidence::kInvalid;
        if (evidence == QuiescenceEvidence::kInvalid) {
          return {
            QuiescenceWaitStatus::kInvalid,
            "malformed JointState while proving quiescence " + phase + ": " +
            evidence_error};
        }
        if (evidence == QuiescenceEvidence::kReady) {
          return {
            QuiescenceWaitStatus::kReady,
            "fresh JointState evidence proved quiescence " + phase};
        }
      }

      if (execution_context.heartbeat() ==
        manipulation::workflow::TransferExecutionControlResult::kStopped)
      {
        return {
          QuiescenceWaitStatus::kCanceled,
          "trajectory maintenance stopped during joint-state quiescence " + phase};
      }
    }
  }

  static PhysicalCommandResult quiescence_failure(
    const QuiescenceWaitResult & result,
    const PhysicalCommandSafety safety)
  {
    switch (result.status) {
      case QuiescenceWaitStatus::kTimedOut:
        return PhysicalCommandResult::failure(
          PhysicalCommandOutcome::kTimedOut, safety, result.detail);
      case QuiescenceWaitStatus::kCanceled:
        return PhysicalCommandResult::failure(
          PhysicalCommandOutcome::kCanceled, safety, result.detail);
      case QuiescenceWaitStatus::kShutdown:
        return PhysicalCommandResult::failure(
          PhysicalCommandOutcome::kShutdown, safety, result.detail);
      case QuiescenceWaitStatus::kInvalid:
        return PhysicalCommandResult::failure(
          PhysicalCommandOutcome::kInternalError, safety, result.detail);
      case QuiescenceWaitStatus::kReady:
        return PhysicalCommandResult::failure(
          PhysicalCommandOutcome::kInternalError, safety,
          "ready joint-state evidence entered failure mapping");
    }
    return PhysicalCommandResult::failure(
      PhysicalCommandOutcome::kInternalError, safety,
      "unknown joint-state quiescence result");
  }

  static AcceptedResult pre_dispatch_failure(
    const QuiescenceWaitResult & result)
  {
    detail::AcceptedPhysicalActionStatus status;
    switch (result.status) {
      case QuiescenceWaitStatus::kTimedOut:
        status = detail::AcceptedPhysicalActionStatus::kInternalError;
        break;
      case QuiescenceWaitStatus::kCanceled:
        status = detail::AcceptedPhysicalActionStatus::kCanceled;
        break;
      case QuiescenceWaitStatus::kShutdown:
        status = detail::AcceptedPhysicalActionStatus::kShutdown;
        break;
      case QuiescenceWaitStatus::kInvalid:
      case QuiescenceWaitStatus::kReady:
        status = detail::AcceptedPhysicalActionStatus::kInternalError;
        break;
    }
    return {
      status, std::nullopt, result.detail,
      detail::ActionDispatchState::kNotSent, false};
  }

  AcceptedResult dispatch_after_quiescence(
    const ExecuteTrajectory::Goal & goal,
    const JointTarget & start_target,
    const std::chrono::steady_clock::duration accepted_motion_timeout,
    manipulation::workflow::TransferExecutionContext & execution_context,
    const std::function<bool()> & stop_requested)
  {
    if (!context_ || !context_->is_valid()) {
      return {
        detail::AcceptedPhysicalActionStatus::kShutdown, std::nullopt,
        "ROS context is shutting down",
        detail::ActionDispatchState::kNotSent, false};
    }
    if (stop_requested() || execution_context.stop_requested()) {
      return {
        detail::AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
        "trajectory execution canceled before discovery",
        detail::ActionDispatchState::kNotSent, false};
    }
    while (!client_->action_server_is_ready()) {
      if (execution_context.checkpoint() ==
        manipulation::workflow::TransferExecutionControlResult::kStopped ||
        stop_requested())
      {
        return {
          detail::AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
          "trajectory execution canceled during discovery",
          detail::ActionDispatchState::kNotSent, false};
      }
      if (!context_->is_valid()) {
        return {
          detail::AcceptedPhysicalActionStatus::kShutdown, std::nullopt,
          "ROS context shut down during trajectory discovery",
          detail::ActionDispatchState::kNotSent, false};
      }
      try {
        (void)client_->wait_for_action_server(kQuiescenceWaitSlice);
      } catch (const std::exception & error) {
        return {
          detail::AcceptedPhysicalActionStatus::kInternalError, std::nullopt,
          error.what(), detail::ActionDispatchState::kNotSent, false};
      }
    }

    for (;; ) {
      std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_);
      const auto start_evidence = wait_for_quiescence(
        start_target, execution_context, stop_requested,
        "before ExecuteTrajectory dispatch");
      if (start_evidence.status != QuiescenceWaitStatus::kReady) {
        return pre_dispatch_failure(start_evidence);
      }

      const auto state = std::make_shared<AcceptedState>(
        client_, accepted_motion_timeout);
      bool installed = false;
      bool stopped_inside_gate = false;
      const auto gate = execution_context.run_operation(
        [&]() {
          // dispatch_mutex_ remains owned from the last quiescent sample through
          // async_send_goal(). The lifecycle gate orders pause/cancel admission.
          if (stop_requested()) {
            stopped_inside_gate = true;
            return;
          }
          installed = active_slot_.install(state);
          if (!installed) {
            return;
          }
          try {
            (void)client_->async_send_goal(goal, state->callbacks());
          } catch (const std::exception & error) {
            state->mark_send_failure(error.what());
            state->request_cancel();
          } catch (...) {
            state->mark_send_failure(
              "unknown exception while sending ExecuteTrajectory");
            state->request_cancel();
          }
        });
      dispatch_lock.unlock();

      if (gate == manipulation::workflow::TaskOperationGateResult::kStopped ||
        stopped_inside_gate)
      {
        return {
          detail::AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
          "trajectory dispatch was stopped before send",
          detail::ActionDispatchState::kNotSent, false};
      }
      if (gate == manipulation::workflow::TaskOperationGateResult::kPaused) {
        if (execution_context.checkpoint() ==
          manipulation::workflow::TransferExecutionControlResult::kStopped)
        {
          return {
            detail::AcceptedPhysicalActionStatus::kCanceled, std::nullopt,
            "trajectory dispatch was stopped while paused",
            detail::ActionDispatchState::kNotSent, false};
        }
        continue;
      }
      if (!installed) {
        return {
          detail::AcceptedPhysicalActionStatus::kUnavailable, std::nullopt,
          "previous ExecuteTrajectory goal has not reached a terminal state",
          detail::ActionDispatchState::kAccepted, false};
      }
      return state->wait(context_, execution_context, stop_requested);
    }
  }

  std::function<bool()> stop_predicate() const
  {
    const auto generation = goal_generation_.load(std::memory_order_acquire);
    return [this, generation]() {
             return stop_generation_.load(std::memory_order_acquire) != generation;
           };
  }

  const RosTrajectoryExecutionPortConfig config_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;
  std::mutex dispatch_mutex_;
  mutable detail::ActiveActionSlot active_slot_;
  mutable std::mutex joint_state_mutex_;
  std::condition_variable joint_state_cv_;
  std::deque<ReceivedJointState> joint_state_queue_;
  std::uint64_t joint_state_sequence_{0U};
  std::atomic_bool convergence_active_{false};
  std::atomic_bool terminal_state_unknown_{false};
  std::atomic<std::uint64_t> stop_generation_{0U};
  std::atomic<std::uint64_t> goal_generation_{0U};
};

class RosStrategyGripperPort::Impl final
{
public:
  Impl(rclcpp::Node & node, RosStrategyGripperPortConfig config)
  : config_(std::move(config)),
    context_(node.get_node_base_interface()->get_context()),
    callback_group_(node.create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    validateRosStrategyGripperPortConfig(config_);
    client_ = rclcpp_action::create_client<GripperCommand>(
      &node, config_.action_name, callback_group_);
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
    if (active_slot_.dispatch_unknown()) {
      return {false, "previous GripperCommand dispatch outcome is unknown"};
    }
    if (active_slot_.busy()) {
      return {false, "previous GripperCommand goal is not terminal"};
    }
    if (!client_->action_server_is_ready()) {
      return {false, "GripperCommand action is unavailable"};
    }
    return {true, "GripperCommand action is ready"};
  }

  StrategyGripperCommandResult command(
    const StrategyGripperCommandRequest & request,
    const std::chrono::steady_clock::duration accepted_command_timeout,
    manipulation::workflow::TransferExecutionContext & execution_context)
  {
    if (!std::isfinite(request.width_m) || request.width_m < 0.0 ||
      !std::isfinite(request.max_effort_n) || request.max_effort_n < 0.0 ||
      accepted_command_timeout <= std::chrono::steady_clock::duration::zero())
    {
      return StrategyGripperCommandResult::failure(
        PhysicalCommandOutcome::kRejected,
        PhysicalCommandSafety::kNotAccepted,
        GripperPostcondition::kNone,
        "gripper width/effort must be finite and non-negative and timeout positive");
    }

    GripperCommand::Goal goal;
    goal.command.position = request.width_m;
    goal.command.max_effort = request.max_effort_n;
    const auto call = detail::call_accepted_physical_action<GripperCommand>(
      client_, goal, accepted_command_timeout, context_,
      execution_context, stop_predicate(), active_slot_,
      dispatch_mutex_);
    return gripper_terminal_result(call, request);
  }

  void request_stop() noexcept
  {
    {
      const std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
      stop_generation_.fetch_add(1U, std::memory_order_acq_rel);
    }
    active_slot_.request_cancel();
  }

private:
  std::function<bool()> stop_predicate() const
  {
    const auto generation = goal_generation_.load(std::memory_order_acquire);
    return [this, generation]() {
             return stop_generation_.load(std::memory_order_acquire) != generation;
           };
  }

  const RosStrategyGripperPortConfig config_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp_action::Client<GripperCommand>::SharedPtr client_;
  std::mutex dispatch_mutex_;
  mutable detail::ActiveActionSlot active_slot_;
  std::atomic<std::uint64_t> stop_generation_{0U};
  std::atomic<std::uint64_t> goal_generation_{0U};
};

RosTrajectoryExecutionPort::RosTrajectoryExecutionPort(
  rclcpp::Node & node,
  RosTrajectoryExecutionPortConfig config)
: impl_(std::make_unique<Impl>(node, std::move(config)))
{
}

RosTrajectoryExecutionPort::~RosTrajectoryExecutionPort() = default;

void RosTrajectoryExecutionPort::begin_goal() noexcept
{
  impl_->begin_goal();
}

PickLiftPlaceReadiness RosTrajectoryExecutionPort::cached_readiness() const
{
  return impl_->cached_readiness();
}

PhysicalCommandResult RosTrajectoryExecutionPort::execute(
  const TrajectoryExecutionRequest & request,
  const std::chrono::steady_clock::duration accepted_motion_timeout,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->execute(request, accepted_motion_timeout, context);
}

void RosTrajectoryExecutionPort::request_stop() noexcept
{
  impl_->request_stop();
}

RosStrategyGripperPort::RosStrategyGripperPort(
  rclcpp::Node & node,
  RosStrategyGripperPortConfig config)
: impl_(std::make_unique<Impl>(node, std::move(config)))
{
}

RosStrategyGripperPort::~RosStrategyGripperPort() = default;

void RosStrategyGripperPort::begin_goal() noexcept
{
  impl_->begin_goal();
}

PickLiftPlaceReadiness RosStrategyGripperPort::cached_readiness() const
{
  return impl_->cached_readiness();
}

StrategyGripperCommandResult RosStrategyGripperPort::command(
  const StrategyGripperCommandRequest & request,
  const std::chrono::steady_clock::duration accepted_command_timeout,
  manipulation::workflow::TransferExecutionContext & context)
{
  return impl_->command(request, accepted_command_timeout, context);
}

void RosStrategyGripperPort::request_stop() noexcept
{
  impl_->request_stop();
}

}  // namespace manipulation::moveit
