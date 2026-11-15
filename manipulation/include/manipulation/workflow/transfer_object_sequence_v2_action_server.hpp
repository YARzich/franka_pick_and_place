// Responsibility: Expose the single versioned high-level transfer lifecycle.

#ifndef MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_ACTION_SERVER_HPP_
#define MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "manipulation/workflow/transfer_sequence_action_types.hpp"
#include "manipulation/workflow/transfer_sequence_task_context.hpp"
#include "manipulation_msgs/action/transfer_object_sequence_v2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace manipulation::workflow
{

/** Deployment-owned endpoints for the single V2 transfer facade. */
struct TransferObjectSequenceV2ActionServerConfig
{
  std::string action_name{"/transfer_object_sequence_v2"};
  std::string pause_service_name{"/set_transfer_object_sequence_v2_paused"};
};

using TransferObjectSequenceV2 = manipulation_msgs::action::TransferObjectSequenceV2;
using TransferObjectSequenceV2AdmissionCallback =
  std::function<TransferSequenceAdmission(const TransferObjectSequenceV2::Goal &)>;
using TransferObjectSequenceV2ExecuteCallback =
  std::function<TransferSequenceExecution(
      const TransferObjectSequenceV2::Goal &,
      TransferSequenceTaskContext &)>;

/** Behavior injected behind the ROS-independent sequence lifecycle. */
struct TransferObjectSequenceV2BackendCallbacks
{
  TransferObjectSequenceV2AdmissionCallback admit;
  TransferObjectSequenceV2ExecuteCallback execute;
  TransferSequenceRequestStopCallback request_stop;
};

/**
 * Own one V2 transfer action, pause service, and managed worker.
 *
 * The lifecycle deliberately has no task-configurable timeout. Its internal
 * heartbeat is an implementation constant, while cancellation and shutdown
 * remain goal-scoped and interrupt every cooperative backend wait.
 */
class TransferObjectSequenceV2ActionServer final
{
public:
  using Action = TransferObjectSequenceV2;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  TransferObjectSequenceV2ActionServer(
    rclcpp::Node & node,
    TransferObjectSequenceV2ActionServerConfig config,
    TransferObjectSequenceV2BackendCallbacks backend);
  ~TransferObjectSequenceV2ActionServer();

  TransferObjectSequenceV2ActionServer(
    const TransferObjectSequenceV2ActionServer &) = delete;
  TransferObjectSequenceV2ActionServer & operator=(
    const TransferObjectSequenceV2ActionServer &) = delete;
  TransferObjectSequenceV2ActionServer(
    TransferObjectSequenceV2ActionServer &&) = delete;
  TransferObjectSequenceV2ActionServer & operator=(
    TransferObjectSequenceV2ActionServer &&) = delete;

  [[nodiscard]] TransferSequenceActionServerSnapshot snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__TRANSFER_OBJECT_SEQUENCE_V2_ACTION_SERVER_HPP_
