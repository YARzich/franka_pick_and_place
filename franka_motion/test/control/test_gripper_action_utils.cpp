// Responsibility: Verifies gripper action clamping, results, timeouts, and cancellation.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include "franka_motion/gripper_action_utils.hpp"

namespace
{
using namespace std::chrono_literals;

class RclcppTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
}  // namespace

TEST_F(RclcppTest, ServerWaitHonorsApplicationStop)
{
  auto node = rclcpp::Node::make_shared("test_gripper_stopped_client");
  auto client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    node, "/test_gripper_action_utils/missing");

  EXPECT_FALSE(
    franka_motion::waitForGripperActionServer(
      client, node->get_logger(), "/test_gripper_action_utils/missing", 1s,
      []() {return true;}));
}

TEST_F(RclcppTest, SendsClampedGripperCommandToActionServer)
{
  const std::string action_name = "/test_gripper_action_utils/gripper_cmd";
  auto server_node = rclcpp::Node::make_shared("test_gripper_action_server");
  auto client_node = rclcpp::Node::make_shared("test_gripper_action_client");
  auto received_position = std::make_shared<std::promise<double>>();
  auto received_future = received_position->get_future();

  auto server = rclcpp_action::create_server<franka_motion::GripperCommandAction>(
    server_node,
    action_name,
    [received_position](
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const franka_motion::GripperCommandAction::Goal> goal)
    {
      received_position->set_value(goal->command.position);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> &)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> goal_handle)
    {
      auto result = std::make_shared<franka_motion::GripperCommandAction::Result>();
      result->position = goal_handle->get_goal()->command.position;
      result->reached_goal = true;
      result->stalled = false;
      goal_handle->succeed(result);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    client_node,
    action_name);
  franka_motion::FrankaJointStateMonitor joint_state_monitor;
  sensor_msgs::msg::JointState finger_state;
  finger_state.name = {"panda_finger_joint1", "panda_finger_joint2"};
  finger_state.position = {0.04, 0.04};
  joint_state_monitor.update(finger_state);

  const bool server_ready =
    franka_motion::waitForGripperActionServer(
      client,
      client_node->get_logger(),
      action_name,
      1s);
  EXPECT_TRUE(server_ready);

  bool command_sent = false;
  if (server_ready) {
    franka_motion::GripperCommandWaitOptions options;
    options.action_timeout = 1s;
    options.state_timeout = 20ms;
    command_sent = franka_motion::sendGripperCommandAndWait(
      client,
      joint_state_monitor,
      client_node->get_logger(),
      0.2,
      "test",
      "test state",
      false,
      options);
  }
  EXPECT_TRUE(command_sent);

  if (command_sent) {
    const auto receive_status = received_future.wait_for(1s);
    EXPECT_EQ(receive_status, std::future_status::ready);
    if (receive_status == std::future_status::ready) {
      EXPECT_DOUBLE_EQ(received_future.get(), 0.08);
    }
  }

  executor.cancel();
  spin_thread.join();
}

TEST_F(RclcppTest, RejectsAbortedAndSemanticallyFailedResults)
{
  const std::string aborted_action = "/test_gripper_action_utils/aborted";
  const std::string stalled_action = "/test_gripper_action_utils/stalled";
  auto server_node = rclcpp::Node::make_shared("test_gripper_failure_server");
  auto client_node = rclcpp::Node::make_shared("test_gripper_failure_client");

  auto accept_goal = [](
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const franka_motion::GripperCommandAction::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
  auto accept_cancel = [](
    const std::shared_ptr<
      rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> &)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    };
  auto aborted_server = rclcpp_action::create_server<franka_motion::GripperCommandAction>(
    server_node,
    aborted_action,
    accept_goal,
    accept_cancel,
    [](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> goal_handle)
    {
      auto result = std::make_shared<franka_motion::GripperCommandAction::Result>();
      goal_handle->abort(result);
    });
  auto stalled_server = rclcpp_action::create_server<franka_motion::GripperCommandAction>(
    server_node,
    stalled_action,
    accept_goal,
    accept_cancel,
    [](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> goal_handle)
    {
      auto result = std::make_shared<franka_motion::GripperCommandAction::Result>();
      result->position = 0.03;
      result->reached_goal = false;
      result->stalled = true;
      goal_handle->succeed(result);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto aborted_client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    client_node, aborted_action);
  auto stalled_client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    client_node, stalled_action);
  ASSERT_TRUE(aborted_client->wait_for_action_server(1s));
  ASSERT_TRUE(stalled_client->wait_for_action_server(1s));

  EXPECT_FALSE(franka_motion::sendGripperCommand(
      aborted_client, 0.04, client_node->get_logger(), "aborted", 10.0, 1s));
  EXPECT_FALSE(franka_motion::sendGripperCommand(
      stalled_client, 0.0, client_node->get_logger(), "stall denied", 10.0, 1s, false));
  EXPECT_FALSE(franka_motion::sendGripperCommand(
      stalled_client, 0.0, client_node->get_logger(), "stall without baseline", 10.0, 1s, true));
  EXPECT_TRUE(franka_motion::sendGripperCommand(
      stalled_client, 0.0, client_node->get_logger(), "stall allowed", 10.0, 1s, true, 0.08));
  EXPECT_FALSE(franka_motion::sendGripperCommand(
      stalled_client, 0.0, client_node->get_logger(), "stall without progress", 10.0, 1s, true,
      0.03));

  executor.cancel();
  spin_thread.join();
}

TEST_F(RclcppTest, CancelsGoalWhenResultTimesOut)
{
  const std::string action_name = "/test_gripper_action_utils/timeout";
  auto server_node = rclcpp::Node::make_shared("test_gripper_timeout_server");
  auto client_node = rclcpp::Node::make_shared("test_gripper_timeout_client");
  auto cancel_received = std::make_shared<std::promise<void>>();
  auto cancel_future = cancel_received->get_future();
  auto active_goal = std::make_shared<std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>>>();

  auto server = rclcpp_action::create_server<franka_motion::GripperCommandAction>(
    server_node,
    action_name,
    [](
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const franka_motion::GripperCommandAction::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [cancel_received](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> &)
    {
      cancel_received->set_value();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [active_goal](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> goal_handle)
    {
      *active_goal = goal_handle;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    client_node, action_name);
  ASSERT_TRUE(client->wait_for_action_server(1s));
  EXPECT_FALSE(franka_motion::sendGripperCommand(
      client, 0.04, client_node->get_logger(), "timeout", 10.0, 100ms));
  EXPECT_EQ(cancel_future.wait_for(1s), std::future_status::ready);

  executor.cancel();
  spin_thread.join();
}

TEST_F(RclcppTest, ApplicationStopCancelsAcceptedGoal)
{
  const std::string action_name = "/test_gripper_action_utils/application_stop";
  auto server_node = rclcpp::Node::make_shared("test_gripper_stop_server");
  auto client_node = rclcpp::Node::make_shared("test_gripper_stop_client");
  auto goal_received = std::make_shared<std::promise<void>>();
  auto goal_received_future = goal_received->get_future();
  auto cancel_received = std::make_shared<std::promise<void>>();
  auto cancel_received_future = cancel_received->get_future();
  auto active_goal = std::make_shared<std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>>>();

  auto server = rclcpp_action::create_server<franka_motion::GripperCommandAction>(
    server_node,
    action_name,
    [](
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const franka_motion::GripperCommandAction::Goal>)
    {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [cancel_received](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> &)
    {
      cancel_received->set_value();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [active_goal, goal_received](
      const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<franka_motion::GripperCommandAction>> goal_handle)
    {
      *active_goal = goal_handle;
      goal_received->set_value();
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});

  auto client = rclcpp_action::create_client<franka_motion::GripperCommandAction>(
    client_node, action_name);
  ASSERT_TRUE(client->wait_for_action_server(1s));
  std::atomic<bool> stop_requested{false};
  auto command_future = std::async(
    std::launch::async,
    [&]() {
      return franka_motion::sendGripperCommand(
        client, 0.04, client_node->get_logger(), "application stop", 10.0, 2s, false,
        std::nullopt, 0.0005, 0.004,
        [&stop_requested]() {return stop_requested.load();});
    });

  ASSERT_EQ(goal_received_future.wait_for(1s), std::future_status::ready);
  stop_requested.store(true);
  EXPECT_FALSE(command_future.get());
  EXPECT_EQ(cancel_received_future.wait_for(1s), std::future_status::ready);

  if (*active_goal && (*active_goal)->is_canceling()) {
    (*active_goal)->canceled(
      std::make_shared<franka_motion::GripperCommandAction::Result>());
  }
  executor.cancel();
  spin_thread.join();
}
