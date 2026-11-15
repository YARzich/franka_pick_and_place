// Responsibility: Verify the unlimited V2 lifecycle, typed waiting feedback, and exact cancel.

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/transfer_object_sequence_v2_action_server.hpp"
#include "manipulation_msgs/msg/manipulation_task_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace manipulation::workflow
{
namespace
{

using namespace std::chrono_literals;
using Action = manipulation_msgs::action::TransferObjectSequenceV2;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

Action::Goal one_step_goal()
{
  Action::Goal goal;
  goal.steps.resize(1U);
  auto & step = goal.steps.front();
  step.object_id = "test_cube";
  step.target_mode = manipulation_msgs::msg::TransferObjectStepV2::TARGET_ABSOLUTE_POSE;
  step.target_object_pose.header.frame_id = "world";
  step.target_object_pose.header.stamp.sec = 1;
  step.target_object_pose.pose.orientation.w = 1.0;
  step.recovery_policy = manipulation_msgs::msg::TransferObjectStepV2::RECOVERY_SAFE_STOP;
  return goal;
}

TransferSequenceAdmission accept_goal(const Action::Goal &)
{
  return {true, manipulation::task::ObjectDisposition::kObjectAtSource, {}};
}

TransferSequenceExecution success_execution()
{
  const auto outcome = manipulation::task::TaskOutcome::make_success(
    manipulation::task::ObjectDisposition::kObjectAtTarget, "V2 step complete");
  return {
    TransferSequenceResult{outcome, 1U, std::nullopt, {outcome}},
    {1ms}};
}

class TransferObjectSequenceV2ActionServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context_);
    server_node_ = std::make_shared<rclcpp::Node>("v2_sequence_test_server", options);
    client_node_ = std::make_shared<rclcpp::Node>("v2_sequence_test_client", options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(
      executor_options);
    executor_->add_node(server_node_);
    executor_->add_node(client_node_);
  }

  void TearDown() override
  {
    executor_->remove_node(client_node_);
    executor_->remove_node(server_node_);
    executor_.reset();
    client_node_.reset();
    server_node_.reset();
    context_->shutdown("V2 action test complete");
    context_.reset();
  }

  TransferObjectSequenceV2ActionServerConfig config(const std::string & suffix) const
  {
    return {
      "/test/transfer_object_sequence_v2/" + suffix,
      "/test/set_transfer_object_sequence_v2_paused/" + suffix};
  }

  bool spin_until(
    const std::function<bool()> & predicate,
    const std::chrono::steady_clock::duration timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }

  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(
  TransferObjectSequenceV2ActionServerTest,
  PublishesTypedWaitAndCompletesWithoutWholeGoalDeadline)
{
  const auto server_config = config("waiting");
  std::atomic_bool release{false};
  TransferObjectSequenceV2ActionServer server(
    *server_node_, server_config,
    TransferObjectSequenceV2BackendCallbacks{
        accept_goal,
        [&release](const Action::Goal &, TransferSequenceTaskContext & context) {
          manipulation::task::TaskStateSnapshot state;
          state.phase = manipulation::task::TaskPhase::kWaitingForResources;
          state.detail = "waiting for observation";
          TransferSequenceFeedbackSnapshot feedback{0U, 1U, 0.1F, state};
          feedback.active_object_id = "test_cube";
          feedback.waiting_reason = TransferWaitingReason::kObservation;
          context.report_feedback(std::move(feedback));
          while (!release.load() && !context.stop_requested()) {
            (void)context.wait_for(5ms);
          }
          return context.stop_requested() ?
                 TransferSequenceExecution{
                   TransferSequenceResult{
                     context.make_stopped_outcome(
              manipulation::task::ObjectDisposition::kObjectAtSource),
                     0U, 0U, {}}, {}} : success_execution();
        },
        [&release](TaskStopCause) {release.store(true);}});

  auto client = rclcpp_action::create_client<Action>(client_node_, server_config.action_name);
  ASSERT_TRUE(client->wait_for_action_server(1s));
  std::optional<Action::Feedback> observed;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&observed](
    const std::shared_ptr<GoalHandle>,
    const std::shared_ptr<const Action::Feedback> feedback) {
      if (feedback->waiting_reason == Action::Feedback::WAITING_OBSERVATION) {
        observed = *feedback;
      }
    };
  auto sent = client->async_send_goal(one_step_goal(), options);
  ASSERT_EQ(executor_->spin_until_future_complete(sent, 1s), rclcpp::FutureReturnCode::SUCCESS);
  const auto handle = sent.get();
  ASSERT_TRUE(handle);
  ASSERT_TRUE(spin_until([&observed]() {return observed.has_value();}));
  std::this_thread::sleep_for(150ms);
  EXPECT_TRUE(server.snapshot().active);
  release.store(true);
  auto result = client->async_get_result(handle);
  ASSERT_EQ(executor_->spin_until_future_complete(result, 2s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(observed);
  EXPECT_EQ(observed->active_object_id, "test_cube");
  EXPECT_EQ(observed->waiting_reason, Action::Feedback::WAITING_OBSERVATION);
}

TEST_F(
  TransferObjectSequenceV2ActionServerTest,
  CancelInterruptsWaitAndPreventsLaterSideEffect)
{
  const auto server_config = config("cancel");
  std::atomic_bool waiting{false};
  std::atomic_bool late_side_effect{false};
  TransferObjectSequenceV2ActionServer server(
    *server_node_, server_config,
    TransferObjectSequenceV2BackendCallbacks{
        accept_goal,
        [&waiting, &late_side_effect](
          const Action::Goal &, TransferSequenceTaskContext & context) {
          waiting.store(true);
          while (!context.stop_requested()) {
            (void)context.wait_for(5ms);
          }
          if (!context.stop_requested()) {
            late_side_effect.store(true);
          }
          const auto outcome = context.make_stopped_outcome(
          manipulation::task::ObjectDisposition::kObjectAtSource);
          return TransferSequenceExecution{
            TransferSequenceResult{outcome, 0U, 0U, {outcome}}, {1ms}};
        },
        {}});

  auto client = rclcpp_action::create_client<Action>(client_node_, server_config.action_name);
  ASSERT_TRUE(client->wait_for_action_server(1s));
  auto sent = client->async_send_goal(one_step_goal());
  ASSERT_EQ(executor_->spin_until_future_complete(sent, 1s), rclcpp::FutureReturnCode::SUCCESS);
  const auto handle = sent.get();
  ASSERT_TRUE(handle);
  ASSERT_TRUE(spin_until([&waiting]() {return waiting.load();}));
  auto canceled = client->async_cancel_goal(handle);
  ASSERT_EQ(
    executor_->spin_until_future_complete(canceled, 1s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto result = client->async_get_result(handle);
  ASSERT_EQ(executor_->spin_until_future_complete(result, 2s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(late_side_effect.load());
}

}  // namespace
}  // namespace manipulation::workflow
