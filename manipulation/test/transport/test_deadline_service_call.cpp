// Responsibility: Verifies bounded service-call status mapping and pending-request cleanup.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <initializer_list>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "manipulation_msgs/srv/get_planning_scene_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation/transport/deadline_service_call.hpp"

namespace manipulation::transport
{
namespace
{

using namespace std::chrono_literals;
using Service = manipulation_msgs::srv::GetPlanningSceneState;

std::uint64_t next_test_id()
{
  static std::atomic<std::uint64_t> next{0U};
  return next.fetch_add(1U, std::memory_order_relaxed);
}

struct TestEndpoints
{
  explicit TestEndpoints(const std::string & test_name)
  : TestEndpoints(test_name, rclcpp::contexts::get_global_default_context())
  {
  }

  TestEndpoints(
    const std::string & test_name,
    std::shared_ptr<rclcpp::Context> node_context)
  : TestEndpoints(test_name, node_context, node_context)
  {
  }

  TestEndpoints(
    const std::string & test_name,
    const std::shared_ptr<rclcpp::Context> & server_context,
    std::shared_ptr<rclcpp::Context> client_context)
  : context(std::move(client_context))
  {
    const auto suffix = test_name + "_" + std::to_string(next_test_id());
    service_name = "/deadline_service_call/" + suffix;
    rclcpp::NodeOptions server_options;
    server_options.context(server_context);
    rclcpp::NodeOptions client_options;
    client_options.context(context);
    server_node = rclcpp::Node::make_shared(
      "deadline_server_" + suffix, server_options);
    client_node = rclcpp::Node::make_shared(
      "deadline_client_" + suffix, client_options);
    client = client_node->create_client<Service>(service_name);
  }

  std::shared_ptr<rclcpp::Context> context;
  std::string service_name;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Client<Service>::SharedPtr client;
};

class ExecutorThread final
{
public:
  explicit ExecutorThread(std::initializer_list<rclcpp::Node::SharedPtr> nodes)
  {
    for (const auto & node : nodes) {
      executor_.add_node(node);
    }
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread thread_;
};

class DeadlineServiceCallTest : public ::testing::Test
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

TEST_F(DeadlineServiceCallTest, RejectsInvalidRequestAndPollPeriod)
{
  TestEndpoints endpoints{"invalid_arguments"};
  const auto deadline = std::chrono::steady_clock::now() + 100ms;

  const auto null_request = call_service_until<Service>(
    *endpoints.client, Service::Request::SharedPtr{}, deadline, 1ms,
    endpoints.context,
    []() {return false;});
  EXPECT_EQ(null_request.status, ServiceCallStatus::kInternalError);
  EXPECT_FALSE(null_request.request_sent);

  const auto zero_poll = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(), deadline, 0ms,
    endpoints.context,
    []() {return false;});
  EXPECT_EQ(zero_poll.status, ServiceCallStatus::kInternalError);
  EXPECT_FALSE(zero_poll.request_sent);
}

TEST_F(DeadlineServiceCallTest, RejectsUnavailableContextBeforeSending)
{
  TestEndpoints endpoints{"unavailable_context"};
  const auto deadline = std::chrono::steady_clock::now() + 100ms;

  const auto null_context = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(), deadline, 1ms,
    std::shared_ptr<rclcpp::Context>{},
    []() {return false;});
  EXPECT_EQ(null_context.status, ServiceCallStatus::kShutdown);
  EXPECT_FALSE(null_context.request_sent);

  const auto uninitialized_context = std::make_shared<rclcpp::Context>();
  const auto invalid_context = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(), deadline, 1ms,
    uninitialized_context,
    []() {return false;});
  EXPECT_EQ(invalid_context.status, ServiceCallStatus::kShutdown);
  EXPECT_FALSE(invalid_context.request_sent);
}

TEST_F(DeadlineServiceCallTest, HonorsCancellationBeforeSendingAReadyServiceRequest)
{
  TestEndpoints endpoints{"pre_send_cancel"};
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [](const Service::Request::SharedPtr, Service::Response::SharedPtr) {});
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));

  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now() + 1s, 1ms, endpoints.context,
    []() {return true;});

  EXPECT_EQ(result.status, ServiceCallStatus::kCanceled);
  EXPECT_EQ(result.detail, "service call canceled before send");
  EXPECT_FALSE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  (void)service;
}

TEST_F(DeadlineServiceCallTest, ReportsUnavailableServiceAtDeadline)
{
  TestEndpoints endpoints{"unavailable"};

  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now(), 1ms, endpoints.context,
    []() {return false;});

  EXPECT_EQ(result.status, ServiceCallStatus::kUnavailable);
  EXPECT_EQ(result.detail, "service unavailable at deadline");
  EXPECT_FALSE(result.request_sent);
}

TEST_F(DeadlineServiceCallTest, DoesNotSendToReadyServiceAfterDeadline)
{
  TestEndpoints endpoints{"ready_expired"};
  std::atomic<std::size_t> received_requests{0U};
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [&received_requests](
      const Service::Request::SharedPtr,
      Service::Response::SharedPtr)
    {
      received_requests.fetch_add(1U, std::memory_order_relaxed);
    });
  ExecutorThread executor{endpoints.server_node, endpoints.client_node};
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));

  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now(), 1ms, endpoints.context,
    []() {return false;});

  EXPECT_EQ(result.status, ServiceCallStatus::kTimeout);
  EXPECT_EQ(result.detail, "service deadline expired before send");
  EXPECT_FALSE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(received_requests.load(std::memory_order_relaxed), 0U);
  (void)service;
}

TEST_F(DeadlineServiceCallTest, ReturnsTypedResponseFromExecutorOwnedClient)
{
  TestEndpoints endpoints{"success"};
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [](
      const Service::Request::SharedPtr,
      Service::Response::SharedPtr response)
    {
      response->state.session_id = "test-session";
      response->state.revision = 7U;
      response->result.success = true;
      response->result.revision = 7U;
    });
  ExecutorThread executor{endpoints.server_node, endpoints.client_node};
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));

  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now() + 1s, 1ms, endpoints.context,
    []() {return false;});

  ASSERT_EQ(result.status, ServiceCallStatus::kSuccess);
  ASSERT_NE(result.response, nullptr);
  EXPECT_EQ(result.response->state.session_id, "test-session");
  EXPECT_EQ(result.response->state.revision, 7U);
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  (void)service;
}

TEST_F(DeadlineServiceCallTest, RemovesPendingRequestAfterResponseTimeout)
{
  TestEndpoints endpoints{"response_timeout"};
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [](const Service::Request::SharedPtr, Service::Response::SharedPtr) {});
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));

  // No executor owns the client, so the sent request remains pending until the
  // helper reaches its steady deadline and removes the corresponding future.
  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now() + 20ms, 1ms, endpoints.context,
    []() {return false;});

  EXPECT_EQ(result.status, ServiceCallStatus::kTimeout);
  EXPECT_EQ(result.detail, "service response deadline exceeded");
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  (void)service;
}

TEST_F(DeadlineServiceCallTest, RemovesPendingRequestAfterPostSendCancellation)
{
  TestEndpoints endpoints{"response_cancel"};
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [](const Service::Request::SharedPtr, Service::Response::SharedPtr) {});
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));
  std::size_t stop_checks = 0U;

  const auto result = call_service_until<Service>(
    *endpoints.client, std::make_shared<Service::Request>(),
    std::chrono::steady_clock::now() + 1s, 1ms, endpoints.context,
    [&stop_checks]() {return ++stop_checks >= 2U;});

  EXPECT_EQ(result.status, ServiceCallStatus::kCanceled);
  EXPECT_EQ(result.detail, "service response wait canceled");
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  (void)service;
}

TEST_F(DeadlineServiceCallTest, RemovesPendingRequestAfterCustomContextShutdown)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  TestEndpoints endpoints{
    "context_shutdown", rclcpp::contexts::get_global_default_context(), context};
  std::promise<void> request_received;
  auto request_received_future = request_received.get_future();
  auto service = endpoints.server_node->create_service<Service>(
    endpoints.service_name,
    [&request_received](
      const Service::Request::SharedPtr,
      Service::Response::SharedPtr)
    {
      request_received.set_value();
      std::this_thread::sleep_for(100ms);
    });
  ExecutorThread executor{endpoints.server_node};
  ASSERT_TRUE(endpoints.client->wait_for_service(1s));

  auto call_future = std::async(std::launch::async, [&endpoints, &context]() {
        return call_service_until<Service>(
        *endpoints.client, std::make_shared<Service::Request>(),
        std::chrono::steady_clock::now() + 1s, 1ms, context,
          []() {return false;});
    });
  ASSERT_EQ(request_received_future.wait_for(1s), std::future_status::ready);
  (void)context->shutdown("deadline service call test");
  ASSERT_EQ(call_future.wait_for(1s), std::future_status::ready);
  const auto result = call_future.get();

  EXPECT_EQ(result.status, ServiceCallStatus::kShutdown);
  EXPECT_EQ(result.detail, "ROS context shut down during service response wait");
  EXPECT_TRUE(result.request_sent);
  EXPECT_EQ(endpoints.client->prune_pending_requests(), 0U);
  EXPECT_TRUE(rclcpp::ok());
  (void)service;
}

}  // namespace
}  // namespace manipulation::transport
