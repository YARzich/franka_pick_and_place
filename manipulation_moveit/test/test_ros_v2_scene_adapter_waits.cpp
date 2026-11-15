// Responsibility: Prove production V2 scene service waits continue, stop, and never dispatch late.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "manipulation_msgs/msg/planning_scene_operation_result.hpp"
#include "manipulation_msgs/srv/acquire_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/apply_planning_scene_diff.hpp"
#include "manipulation_msgs/srv/get_planning_scene_state.hpp"
#include "manipulation_msgs/srv/release_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/renew_planning_scene_lease.hpp"
#include "manipulation_msgs/srv/validate_planning_scene_lease.hpp"
#include "moveit/utils/robot_model_test_utils.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"

#include "manipulation/workflow/object_catalog.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation_moveit/ros_v2_scene_adapter.hpp"

namespace manipulation::moveit
{
namespace
{

using namespace std::chrono_literals;
using Acquire = manipulation_msgs::srv::AcquirePlanningSceneLease;
using Apply = manipulation_msgs::srv::ApplyPlanningSceneDiff;
using ErrorCode = manipulation::task::TaskErrorCode;
using GetScene = moveit_msgs::srv::GetPlanningScene;
using GetState = manipulation_msgs::srv::GetPlanningSceneState;
using Instance = manipulation::workflow::ObjectInstance;
using OperationResult = manipulation_msgs::msg::PlanningSceneOperationResult;
using Release = manipulation_msgs::srv::ReleasePlanningSceneLease;
using Renew = manipulation_msgs::srv::RenewPlanningSceneLease;
using Validate = manipulation_msgs::srv::ValidatePlanningSceneLease;

constexpr auto kMustRemainPending = 120ms;
constexpr auto kFormerResponseDeadlineEvidence = 5100ms;
constexpr auto kFastStopWindow = 1s;
constexpr auto kNoLateDispatchWindow = 160ms;
// DDS discovery is fixture setup, not the cancellation response under test.
// New contexts can need a discovery retry after a previous case shuts down.
constexpr auto kServiceDiscoveryWindow = 10s;

std::uint64_t next_test_id()
{
  static std::atomic<std::uint64_t> next{0U};
  return next.fetch_add(1U, std::memory_order_relaxed);
}

class OneShotSignal final
{
public:
  void notify()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ready_ = true;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_for(const std::chrono::steady_clock::duration timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this]() {return ready_;});
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool ready_{false};
};

class ResponseGate final
{
public:
  void wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {return open_;});
  }

  void open()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    cv_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool open_{false};
};

class OpenGateOnExit final
{
public:
  explicit OpenGateOnExit(std::shared_ptr<ResponseGate> gate)
  : gate_(std::move(gate))
  {
  }

  ~OpenGateOnExit()
  {
    gate_->open();
  }

private:
  std::shared_ptr<ResponseGate> gate_;
};

class TestExecutionContext
  : public manipulation::workflow::TransferExecutionContext
{
public:
  manipulation::workflow::TransferExecutionControlResult checkpoint() override
  {
    return control_result();
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load(std::memory_order_acquire);
  }

  manipulation::workflow::TransferExecutionControlResult wait_until(
    const TimePoint deadline) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    (void)cv_.wait_until(
      lock, deadline, [this]() {return stopped_.load(std::memory_order_acquire);});
    return control_result();
  }

  manipulation::workflow::TransferExecutionControlResult heartbeat() override
  {
    return control_result();
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void report_waiting(
    manipulation::workflow::TransferWaitingReason,
    manipulation::task::TaskStateSnapshot) override
  {
    waiting_reported_.notify();
  }

  void cancel() noexcept
  {
    stopped_.store(true, std::memory_order_release);
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_for_waiting_report(
    const std::chrono::steady_clock::duration timeout)
  {
    return waiting_reported_.wait_for(timeout);
  }

private:
  [[nodiscard]] manipulation::workflow::TransferExecutionControlResult
  control_result() const noexcept
  {
    return stop_requested() ?
           manipulation::workflow::TransferExecutionControlResult::kStopped :
           manipulation::workflow::TransferExecutionControlResult::kContinue;
  }

  std::atomic_bool stopped_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
  OneShotSignal waiting_reported_;
};

class ShutdownAtDispatchContext final : public TestExecutionContext
{
public:
  explicit ShutdownAtDispatchContext(std::shared_ptr<rclcpp::Context> context)
  : context_(std::move(context))
  {
  }

  manipulation::workflow::TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    (void)context_->shutdown("scene dispatch boundary shutdown test");
    operation();
    return manipulation::workflow::TaskOperationGateResult::kExecuted;
  }

private:
  std::shared_ptr<rclcpp::Context> context_;
};

class ContextExecutorThread final
{
public:
  ContextExecutorThread(
    const std::shared_ptr<rclcpp::Context> & context,
    std::initializer_list<rclcpp::Node::SharedPtr> nodes)
  {
    rclcpp::ExecutorOptions options;
    options.context = context;
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      options, 4U);
    for (const auto & node : nodes) {
      executor_->add_node(node);
    }
    thread_ = std::thread([this]() {executor_->spin();});
  }

  ~ContextExecutorThread()
  {
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread thread_;
};

class SceneServiceHarness final
{
public:
  SceneServiceHarness(
    rclcpp::Node & node,
    RosV2SceneAdapterConfig config)
  : node_(node), config_(std::move(config))
  {
  }

  void start(std::shared_ptr<ResponseGate> acquire_response_gate = {})
  {
    acquire_service_ = node_.create_service<Acquire>(
      config_.acquire_service,
      [this, gate = std::move(acquire_response_gate)](
        const Acquire::Request::SharedPtr,
        Acquire::Response::SharedPtr response)
      {
        acquire_requests_.fetch_add(1U, std::memory_order_relaxed);
        acquire_side_effects_.fetch_add(1U, std::memory_order_relaxed);
        acquire_received_.notify();
        if (gate) {
          gate->wait();
        }
        response->lease_id = "test-lease";
        response->granted_ttl.sec = 45;
        set_success(response->result);
        acquire_response_returned_.notify();
      });

    state_service_ = node_.create_service<GetState>(
      config_.state_service,
      [this](
        const GetState::Request::SharedPtr,
        GetState::Response::SharedPtr response)
      {
        state_requests_.fetch_add(1U, std::memory_order_relaxed);
        response->state.session_id = "test-session";
        response->state.revision = 7U;
        response->state.backend_service_ready = true;
        set_success(response->result);
      });

    validate_service_ = node_.create_service<Validate>(
      config_.validate_service,
      [this](
        const Validate::Request::SharedPtr,
        Validate::Response::SharedPtr response)
      {
        validate_requests_.fetch_add(1U, std::memory_order_relaxed);
        response->session_id = "test-session";
        response->remaining_ttl.sec = 30;
        set_success(response->result);
      });

    get_scene_service_ = node_.create_service<GetScene>(
      config_.get_planning_scene_service,
      [this](
        const GetScene::Request::SharedPtr,
        GetScene::Response::SharedPtr response)
      {
        get_scene_requests_.fetch_add(1U, std::memory_order_relaxed);
        response->scene.is_diff = false;
        response->scene.robot_state.is_diff = false;
      });

    apply_service_ = node_.create_service<Apply>(
      config_.apply_diff_service,
      [this](
        const Apply::Request::SharedPtr,
        Apply::Response::SharedPtr response)
      {
        apply_requests_.fetch_add(1U, std::memory_order_relaxed);
        set_success(response->result);
      });

    renew_service_ = node_.create_service<Renew>(
      config_.renew_service,
      [this](
        const Renew::Request::SharedPtr,
        Renew::Response::SharedPtr response)
      {
        renew_requests_.fetch_add(1U, std::memory_order_relaxed);
        response->granted_ttl.sec = 45;
        set_success(response->result);
      });

    release_service_ = node_.create_service<Release>(
      config_.release_service,
      [this](
        const Release::Request::SharedPtr,
        Release::Response::SharedPtr response)
      {
        release_requests_.fetch_add(1U, std::memory_order_relaxed);
        set_success(response->result);
      });
  }

  [[nodiscard]] bool wait_for_acquire(
    const std::chrono::steady_clock::duration timeout)
  {
    return acquire_received_.wait_for(timeout);
  }

  [[nodiscard]] bool wait_for_acquire_response_return(
    const std::chrono::steady_clock::duration timeout)
  {
    return acquire_response_returned_.wait_for(timeout);
  }

  [[nodiscard]] std::size_t acquire_requests() const noexcept
  {
    return acquire_requests_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t acquire_side_effects() const noexcept
  {
    return acquire_side_effects_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t state_requests() const noexcept
  {
    return state_requests_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t validate_requests() const noexcept
  {
    return validate_requests_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t get_scene_requests() const noexcept
  {
    return get_scene_requests_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] std::size_t later_requests() const noexcept
  {
    return state_requests() + validate_requests() + get_scene_requests() +
           apply_requests_.load(std::memory_order_relaxed) +
           renew_requests_.load(std::memory_order_relaxed) +
           release_requests_.load(std::memory_order_relaxed);
  }

private:
  static void set_success(OperationResult & result)
  {
    result.success = true;
    result.error_code = OperationResult::NONE;
    result.revision = 7U;
  }

  rclcpp::Node & node_;
  const RosV2SceneAdapterConfig config_;
  rclcpp::Service<Acquire>::SharedPtr acquire_service_;
  rclcpp::Service<GetState>::SharedPtr state_service_;
  rclcpp::Service<Validate>::SharedPtr validate_service_;
  rclcpp::Service<GetScene>::SharedPtr get_scene_service_;
  rclcpp::Service<Apply>::SharedPtr apply_service_;
  rclcpp::Service<Renew>::SharedPtr renew_service_;
  rclcpp::Service<Release>::SharedPtr release_service_;
  std::atomic_size_t acquire_requests_{0U};
  std::atomic_size_t acquire_side_effects_{0U};
  std::atomic_size_t state_requests_{0U};
  std::atomic_size_t validate_requests_{0U};
  std::atomic_size_t get_scene_requests_{0U};
  std::atomic_size_t apply_requests_{0U};
  std::atomic_size_t renew_requests_{0U};
  std::atomic_size_t release_requests_{0U};
  OneShotSignal acquire_received_;
  OneShotSignal acquire_response_returned_;
};

class StopAcquireOnExit final
{
public:
  StopAcquireOnExit(
    TestExecutionContext & context,
    RosV2SceneAdapter & adapter)
  : context_(context), adapter_(adapter)
  {
  }

  ~StopAcquireOnExit()
  {
    context_.cancel();
    adapter_.request_stop();
  }

private:
  TestExecutionContext & context_;
  RosV2SceneAdapter & adapter_;
};

RosV2SceneAdapterConfig scene_config(const std::string & prefix)
{
  RosV2SceneAdapterConfig config;
  config.owner_id = "scene_wait_test";
  config.attachment_link = "panda_hand";
  config.touch_links = {"panda_hand"};
  config.acquire_service = prefix + "/acquire";
  config.renew_service = prefix + "/renew";
  config.release_service = prefix + "/release";
  config.validate_service = prefix + "/validate";
  config.state_service = prefix + "/state";
  config.apply_diff_service = prefix + "/apply";
  config.get_planning_scene_service = prefix + "/get_scene";
  return config;
}

std::vector<Instance> test_instances()
{
  return {Instance::create("test-object", "test-profile")};
}

class RosV2SceneAdapterWaitTest : public ::testing::Test
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

  void SetUp() override
  {
    const auto id = "case_" + std::to_string(next_test_id());
    const auto prefix = "/test_v2_scene_adapter_waits/" + id;
    config_ = scene_config(prefix);

    client_context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions client_init_options;
    client_init_options.auto_initialize_logging(false);
    client_context_->init(0, nullptr, client_init_options);
    server_context_ = rclcpp::contexts::get_global_default_context();

    rclcpp::NodeOptions client_options;
    client_options.context(client_context_);
    rclcpp::NodeOptions server_options;
    server_options.context(server_context_);
    client_node_ = std::make_shared<rclcpp::Node>(
      "scene_wait_client_" + id, client_options);
    server_node_ = std::make_shared<rclcpp::Node>(
      "scene_wait_server_" + id, server_options);
    client_executor_ = std::make_unique<ContextExecutorThread>(
      client_context_, std::initializer_list<rclcpp::Node::SharedPtr>{client_node_});

    robot_model_ = ::moveit::core::loadTestingRobotModel("panda");
    ASSERT_NE(robot_model_, nullptr);
    adapter_ = std::make_unique<RosV2SceneAdapter>(
      *client_node_, robot_model_, config_);
    services_ = std::make_unique<SceneServiceHarness>(*server_node_, config_);
  }

  void TearDown() override
  {
    adapter_.reset();
    client_executor_.reset();
    server_executor_.reset();
    services_.reset();
    client_node_.reset();
    server_node_.reset();
    robot_model_.reset();
    if (client_context_ && client_context_->is_valid()) {
      (void)client_context_->shutdown("V2 scene adapter wait test complete");
    }
    client_context_.reset();
    server_context_.reset();
  }

  void start_server(std::shared_ptr<ResponseGate> response_gate = {})
  {
    services_->start(std::move(response_gate));
    server_executor_ = std::make_unique<ContextExecutorThread>(
      server_context_, std::initializer_list<rclcpp::Node::SharedPtr>{server_node_});
  }

  [[nodiscard]] bool wait_for_all_services(
    const std::chrono::steady_clock::duration timeout = kServiceDiscoveryWindow) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (adapter_->cached_readiness().ready) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return adapter_->cached_readiness().ready;
  }

  [[nodiscard]] std::future<manipulation::workflow::PlanningSceneLeaseResult>
  start_acquire()
  {
    return std::async(std::launch::async, [this]() {
               return adapter_->acquire(test_instances(), execution_context_);
           });
  }

  RosV2SceneAdapterConfig config_;
  std::shared_ptr<rclcpp::Context> client_context_;
  std::shared_ptr<rclcpp::Context> server_context_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  std::unique_ptr<ContextExecutorThread> client_executor_;
  std::unique_ptr<ContextExecutorThread> server_executor_;
  ::moveit::core::RobotModelConstPtr robot_model_;
  std::unique_ptr<RosV2SceneAdapter> adapter_;
  std::unique_ptr<SceneServiceHarness> services_;
  TestExecutionContext execution_context_;
};

TEST_F(
  RosV2SceneAdapterWaitTest,
  DelayedDiscoveryAndResponseEventuallyContinueWithCompleteScene)
{
  auto response_gate = std::make_shared<ResponseGate>();
  auto result_future = start_acquire();
  StopAcquireOnExit stop_guard{execution_context_, *adapter_};
  OpenGateOnExit gate_guard{response_gate};

  ASSERT_TRUE(execution_context_.wait_for_waiting_report(kFastStopWindow));
  EXPECT_EQ(result_future.wait_for(kMustRemainPending), std::future_status::timeout);

  start_server(response_gate);
  ASSERT_TRUE(wait_for_all_services());
  ASSERT_TRUE(services_->wait_for_acquire(kFastStopWindow));
  EXPECT_EQ(
    result_future.wait_for(kFormerResponseDeadlineEvidence),
    std::future_status::timeout);

  response_gate->open();
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  const auto result = result_future.get();

  ASSERT_TRUE(result.succeeded()) << result.detail();
  ASSERT_TRUE(result.value().has_value());
  EXPECT_EQ(
    result.value()->snapshot().coordinator_session_id(), "test-session");
  EXPECT_EQ(result.value()->snapshot().revision(), 7U);
  EXPECT_EQ(services_->acquire_requests(), 1U);
  EXPECT_EQ(services_->acquire_side_effects(), 1U);
  EXPECT_EQ(services_->state_requests(), 2U);
  EXPECT_EQ(services_->validate_requests(), 1U);
  EXPECT_EQ(services_->get_scene_requests(), 1U);
}

TEST_F(
  RosV2SceneAdapterWaitTest,
  CancelStopsPendingDiscoveryWithoutLateSceneRequestOrSideEffect)
{
  auto result_future = start_acquire();
  StopAcquireOnExit stop_guard{execution_context_, *adapter_};

  ASSERT_TRUE(execution_context_.wait_for_waiting_report(kFastStopWindow));
  EXPECT_EQ(result_future.wait_for(kMustRemainPending), std::future_status::timeout);

  execution_context_.cancel();
  adapter_->request_stop();
  ASSERT_EQ(result_future.wait_for(kFastStopWindow), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.error_code(), ErrorCode::kCanceled);

  start_server();
  ASSERT_TRUE(wait_for_all_services());
  std::this_thread::sleep_for(kNoLateDispatchWindow);
  EXPECT_EQ(services_->acquire_requests(), 0U);
  EXPECT_EQ(services_->acquire_side_effects(), 0U);
  EXPECT_EQ(services_->later_requests(), 0U);
}

TEST_F(
  RosV2SceneAdapterWaitTest,
  ShutdownStopsPendingDiscoveryWithoutLateSceneRequestOrSideEffect)
{
  auto result_future = start_acquire();
  StopAcquireOnExit stop_guard{execution_context_, *adapter_};

  ASSERT_TRUE(execution_context_.wait_for_waiting_report(kFastStopWindow));
  EXPECT_EQ(result_future.wait_for(kMustRemainPending), std::future_status::timeout);

  ASSERT_TRUE(client_context_->shutdown("pending scene discovery shutdown test"));
  ASSERT_EQ(result_future.wait_for(kFastStopWindow), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.error_code(), ErrorCode::kCanceled);
  EXPECT_NE(result.detail().find("shut down"), std::string::npos);

  start_server();
  std::this_thread::sleep_for(kNoLateDispatchWindow);
  EXPECT_EQ(services_->acquire_requests(), 0U);
  EXPECT_EQ(services_->acquire_side_effects(), 0U);
  EXPECT_EQ(services_->later_requests(), 0U);
}

TEST_F(
  RosV2SceneAdapterWaitTest,
  CancelStopsPendingResponseWithoutFollowOnSceneRequestOrSideEffect)
{
  auto response_gate = std::make_shared<ResponseGate>();
  start_server(response_gate);
  ASSERT_TRUE(wait_for_all_services());
  auto result_future = start_acquire();
  StopAcquireOnExit stop_guard{execution_context_, *adapter_};
  OpenGateOnExit gate_guard{response_gate};

  ASSERT_TRUE(services_->wait_for_acquire(kFastStopWindow));
  ASSERT_EQ(services_->acquire_side_effects(), 1U);
  EXPECT_EQ(result_future.wait_for(kMustRemainPending), std::future_status::timeout);

  execution_context_.cancel();
  adapter_->request_stop();
  ASSERT_EQ(result_future.wait_for(kFastStopWindow), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.error_code(), ErrorCode::kCanceled);
  EXPECT_EQ(services_->later_requests(), 0U);

  response_gate->open();
  ASSERT_TRUE(
    services_->wait_for_acquire_response_return(kFastStopWindow));
  std::this_thread::sleep_for(kNoLateDispatchWindow);
  EXPECT_EQ(services_->acquire_requests(), 1U);
  EXPECT_EQ(services_->acquire_side_effects(), 1U);
  EXPECT_EQ(services_->later_requests(), 0U);
}

TEST_F(
  RosV2SceneAdapterWaitTest,
  ShutdownStopsPendingResponseWithoutFollowOnSceneRequestOrSideEffect)
{
  auto response_gate = std::make_shared<ResponseGate>();
  start_server(response_gate);
  ASSERT_TRUE(wait_for_all_services());
  auto result_future = start_acquire();
  StopAcquireOnExit stop_guard{execution_context_, *adapter_};
  OpenGateOnExit gate_guard{response_gate};

  ASSERT_TRUE(services_->wait_for_acquire(kFastStopWindow));
  ASSERT_EQ(services_->acquire_side_effects(), 1U);
  EXPECT_EQ(result_future.wait_for(kMustRemainPending), std::future_status::timeout);

  ASSERT_TRUE(client_context_->shutdown("pending scene response shutdown test"));
  ASSERT_EQ(result_future.wait_for(kFastStopWindow), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.error_code(), ErrorCode::kCanceled);
  EXPECT_NE(result.detail().find("shut down"), std::string::npos);
  EXPECT_EQ(services_->later_requests(), 0U);

  response_gate->open();
  ASSERT_TRUE(
    services_->wait_for_acquire_response_return(kFastStopWindow));
  std::this_thread::sleep_for(kNoLateDispatchWindow);
  EXPECT_EQ(services_->acquire_requests(), 1U);
  EXPECT_EQ(services_->acquire_side_effects(), 1U);
  EXPECT_EQ(services_->later_requests(), 0U);
}

TEST_F(
  RosV2SceneAdapterWaitTest,
  ReadyEndpointShutdownAtDispatchDoesNotSendSceneRequest)
{
  start_server();
  ASSERT_TRUE(wait_for_all_services());
  ShutdownAtDispatchContext shutdown_context{client_context_};

  const auto result = adapter_->acquire(test_instances(), shutdown_context);

  EXPECT_FALSE(result.succeeded());
  EXPECT_EQ(result.error_code(), ErrorCode::kCanceled);
  EXPECT_NE(result.detail().find("shut down before send"), std::string::npos);
  std::this_thread::sleep_for(kNoLateDispatchWindow);
  EXPECT_EQ(services_->acquire_requests(), 0U);
  EXPECT_EQ(services_->acquire_side_effects(), 0U);
  EXPECT_EQ(services_->later_requests(), 0U);
}

}  // namespace
}  // namespace manipulation::moveit
