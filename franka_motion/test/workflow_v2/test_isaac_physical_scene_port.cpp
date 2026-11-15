// Responsibility: Verify the Franka-only Isaac physical/PlanningScene synchronization barrier.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "manipulation/workflow/task_stop_cause.hpp"
#include "manipulation/workflow/transfer_execution_context.hpp"
#include "manipulation_moveit/pick_lift_place_ports.hpp"
#include "workflow_v2/isaac_physical_scene_port.hpp"

namespace
{

using namespace std::chrono_literals;
using franka_motion::workflow_v2::IsaacPhysicalScenePort;
using franka_motion::workflow_v2::IsaacPhysicalScenePortConfig;
using manipulation::moveit::CoordinatedSceneOutcome;
using manipulation::moveit::CoordinatedSceneResult;
using manipulation::moveit::PickLiftPlaceReadiness;
using manipulation::moveit::StrategySceneAttachRequest;
using manipulation::moveit::StrategySceneDetachRequest;
using manipulation::moveit::StrategySceneRevalidationRequest;
using manipulation::workflow::PlanningSceneAccessProof;
using manipulation::workflow::PlanningSceneLease;
using manipulation::workflow::PlanningSceneSnapshot;
using manipulation::workflow::TaskStopCause;
using manipulation::workflow::TaskOperationGateResult;
using manipulation::workflow::TransferExecutionContext;
using manipulation::workflow::TransferExecutionControlResult;
using SetBool = std_srvs::srv::SetBool;

std::uint64_t next_test_id()
{
  static std::atomic<std::uint64_t> next{0U};
  return next.fetch_add(1U, std::memory_order_relaxed);
}

std::string unique_name(const std::string & stem)
{
  return stem + "_" + std::to_string(next_test_id());
}

PlanningSceneSnapshot snapshot(const std::uint64_t revision = 1U)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = false;
  scene.robot_state.is_diff = false;
  return PlanningSceneSnapshot::create("test-session", revision, std::move(scene));
}

PlanningSceneAccessProof proof(const std::uint64_t revision = 1U)
{
  return PlanningSceneAccessProof::create("test-lease", "test-session", revision);
}

PlanningSceneLease next_lease(const std::uint64_t revision)
{
  return PlanningSceneLease::create(snapshot(revision), proof(revision));
}

StrategySceneAttachRequest attach_request(const std::string & object_id = "cube")
{
  return {
    snapshot(), proof(), object_id, "panda_hand",
    {"panda_hand", "panda_leftfinger", "panda_rightfinger"}};
}

StrategySceneDetachRequest detach_request(const std::string & object_id = "cube")
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "panda_link0";
  target.pose.orientation.w = 1.0;
  return {snapshot(), proof(), object_id, "panda_hand", std::move(target)};
}

class TestContext : public TransferExecutionContext
{
public:
  TransferExecutionControlResult checkpoint() override
  {
    return control_result();
  }

  [[nodiscard]] bool stop_requested() const noexcept override
  {
    return stopped_.load(std::memory_order_acquire);
  }

  TransferExecutionControlResult wait_until(const TimePoint deadline) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    (void)cv_.wait_until(lock, deadline, [this]() {return stop_requested();});
    return control_result();
  }

  TransferExecutionControlResult heartbeat() override
  {
    return control_result();
  }

  [[nodiscard]] TaskStopCause stop_cause() const noexcept override
  {
    return stop_requested() ? TaskStopCause::kClientCanceled : TaskStopCause::kNone;
  }

  void report_feedback(manipulation::task::TaskStateSnapshot) override {}

  void cancel() noexcept
  {
    stopped_.store(true, std::memory_order_release);
    cv_.notify_all();
  }

private:
  [[nodiscard]] TransferExecutionControlResult control_result() const noexcept
  {
    return stop_requested() ?
           TransferExecutionControlResult::kStopped :
           TransferExecutionControlResult::kContinue;
  }

  std::atomic_bool stopped_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
};

class StopAtDispatchGateContext final : public TestContext
{
public:
  TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    operation();
    cancel();
    return TaskOperationGateResult::kStopped;
  }
};

class ReentrantContextQueryDetectingGateContext final : public TestContext
{
public:
  [[nodiscard]] bool stop_requested() const noexcept override
  {
    if (inside_gate_.load(std::memory_order_acquire)) {
      reentrant_query_count_.fetch_add(1U, std::memory_order_relaxed);
    }
    return TestContext::stop_requested();
  }

  TaskOperationGateResult run_operation(
    const std::function<void()> & operation) override
  {
    inside_gate_.store(true, std::memory_order_release);
    operation();
    inside_gate_.store(false, std::memory_order_release);
    return TaskOperationGateResult::kExecuted;
  }

  [[nodiscard]] std::size_t reentrant_query_count() const noexcept
  {
    return reentrant_query_count_.load(std::memory_order_relaxed);
  }

private:
  std::atomic_bool inside_gate_{false};
  mutable std::atomic<std::size_t> reentrant_query_count_{0U};
};

class FakeScenePort final : public manipulation::moveit::IPickLiftPlaceScenePort
{
public:
  void begin_goal() override
  {
    ++begin_count;
  }

  CoordinatedSceneResult revalidate(
    const StrategySceneRevalidationRequest &,
    TransferExecutionContext &) override
  {
    ++revalidate_count;
    return CoordinatedSceneResult::success(next_lease(2U), "revalidated");
  }

  CoordinatedSceneResult attach(
    const StrategySceneAttachRequest &,
    TransferExecutionContext &) override
  {
    ++attach_count;
    attach_observed_barrier = barrier_confirmed.load(std::memory_order_acquire);
    if (throw_attach) {
      throw std::runtime_error("fake scene attach exception");
    }
    if (fail_attach) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kConflict, "fake scene attach failed");
    }
    return CoordinatedSceneResult::success(next_lease(2U), "attached");
  }

  CoordinatedSceneResult detach(
    const StrategySceneDetachRequest &,
    TransferExecutionContext &) override
  {
    ++detach_count;
    detach_observed_barrier = barrier_confirmed.load(std::memory_order_acquire);
    if (fail_detach) {
      return CoordinatedSceneResult::failure(
        CoordinatedSceneOutcome::kFailed, "fake scene detach failed");
    }
    return CoordinatedSceneResult::success(next_lease(2U), "detached");
  }

  void request_stop() noexcept override
  {
    ++stop_count;
  }

  std::atomic_bool barrier_confirmed{false};
  bool fail_attach{false};
  bool fail_detach{false};
  bool throw_attach{false};
  bool attach_observed_barrier{false};
  bool detach_observed_barrier{false};
  std::size_t begin_count{0U};
  std::size_t revalidate_count{0U};
  std::size_t attach_count{0U};
  std::size_t detach_count{0U};
  std::size_t stop_count{0U};
};

class ExecutorThread final
{
public:
  explicit ExecutorThread(std::initializer_list<rclcpp::Node::SharedPtr> nodes)
  {
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, 2U);
    for (const auto & node : nodes) {
      executor_->add_node(node);
    }
    thread_ = std::thread([this]() {executor_->spin();});
  }

  ~ExecutorThread()
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

class ResponseGate final
{
public:
  void mark_entered()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    entered_ = true;
    cv_.notify_all();
  }

  [[nodiscard]] bool wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 2s, [this]() {return entered_;});
  }

  void wait_until_released()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() {return released_;});
  }

  void release()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    cv_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool entered_{false};
  bool released_{false};
};

class IsaacPhysicalScenePortTest : public ::testing::Test
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

TEST_F(IsaacPhysicalScenePortTest, EmptyEndpointIsAnExactScenePassthrough)
{
  auto node = std::make_shared<rclcpp::Node>(unique_name("physical_passthrough"));
  FakeScenePort scene;
  IsaacPhysicalScenePort port(
    *node, scene, IsaacPhysicalScenePortConfig{},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;

  port.begin_goal();
  const auto attached = port.attach(attach_request(), context);
  const auto detached = port.detach(detach_request(), context);

  EXPECT_TRUE(attached.succeeded());
  EXPECT_TRUE(detached.succeeded());
  EXPECT_EQ(scene.begin_count, 1U);
  EXPECT_EQ(scene.attach_count, 1U);
  EXPECT_EQ(scene.detach_count, 1U);
  EXPECT_TRUE(port.cached_readiness().ready);
}

TEST_F(IsaacPhysicalScenePortTest, ConfirmedPhysicalAttachPrecedesRawSceneMutation)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("physical_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("physical_server"));
  const auto service_name = "/" + unique_name("set_pick_cube_attached");
  FakeScenePort scene;
  scene.fail_attach = true;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [&scene](
      const SetBool::Request::SharedPtr request,
      SetBool::Response::SharedPtr response)
    {
      scene.barrier_confirmed.store(request->data, std::memory_order_release);
      response->success = true;
      response->message = "physics step confirmed";
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  port.begin_goal();

  const auto result = port.attach(attach_request(), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.attach_count, 1U);
  EXPECT_TRUE(scene.attach_observed_barrier);
  EXPECT_FALSE(port.cached_readiness().ready);
}

TEST_F(IsaacPhysicalScenePortTest, DispatchGateDoesNotReenterExecutionContext)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("non_reentrant_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("non_reentrant_server"));
  const auto service_name = "/" + unique_name("non_reentrant_pick_cube");
  FakeScenePort scene;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [&scene](
      const SetBool::Request::SharedPtr request,
      SetBool::Response::SharedPtr response)
    {
      scene.barrier_confirmed.store(request->data, std::memory_order_release);
      response->success = true;
      response->message = "physics step confirmed";
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  ReentrantContextQueryDetectingGateContext context;
  port.begin_goal();

  const auto result = port.attach(attach_request(), context);

  EXPECT_TRUE(result.succeeded());
  EXPECT_EQ(scene.attach_count, 1U);
  EXPECT_TRUE(scene.attach_observed_barrier);
  EXPECT_EQ(context.reentrant_query_count(), 0U);
}

TEST_F(IsaacPhysicalScenePortTest, ServiceFailureQuarantinesUnknownPhysicalState)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("rejected_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("rejected_server"));
  const auto service_name = "/" + unique_name("reject_pick_cube_attached");
  FakeScenePort scene;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [](
      const SetBool::Request::SharedPtr,
      SetBool::Response::SharedPtr response)
    {
      response->success = false;
      response->message = "fixed joint activation failed";
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  port.begin_goal();

  const auto result = port.attach(attach_request(), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.attach_count, 0U);
  EXPECT_NE(result.detail().find("fixed joint activation failed"), std::string::npos);
  EXPECT_FALSE(port.cached_readiness().ready);

  port.begin_goal();
  const auto quarantined = port.attach(attach_request(), context);
  EXPECT_EQ(quarantined.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.attach_count, 0U);
}

TEST_F(IsaacPhysicalScenePortTest, RawSceneExceptionAfterAckAlsoStaysQuarantined)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("throwing_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("throwing_server"));
  const auto service_name = "/" + unique_name("throwing_pick_cube");
  FakeScenePort scene;
  scene.throw_attach = true;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [&scene](
      const SetBool::Request::SharedPtr request,
      SetBool::Response::SharedPtr response)
    {
      scene.barrier_confirmed.store(request->data, std::memory_order_release);
      response->success = true;
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  port.begin_goal();

  const auto result = port.attach(attach_request(), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_NE(result.detail().find("fake scene attach exception"), std::string::npos);
  EXPECT_FALSE(port.cached_readiness().ready);
}

TEST_F(IsaacPhysicalScenePortTest, ConfiguredEndpointRejectsUnknownObjectBeforeDispatch)
{
  auto node = std::make_shared<rclcpp::Node>(unique_name("unknown_object"));
  FakeScenePort scene;
  const auto service_name = "/" + unique_name("unused_physical_attachment");
  IsaacPhysicalScenePort port(
    *node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  EXPECT_FALSE(port.cached_readiness().ready);
  port.begin_goal();

  const auto result = port.attach(attach_request("other_cube"), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kFailed);
  EXPECT_EQ(scene.attach_count, 0U);
}

TEST_F(IsaacPhysicalScenePortTest, StopAfterDispatchReportsStateUncertainWithoutRawMutation)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("stopped_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("stopped_server"));
  const auto service_name = "/" + unique_name("blocked_pick_cube_attached");
  auto gate = std::make_shared<ResponseGate>();
  FakeScenePort scene;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [gate](
      const SetBool::Request::SharedPtr,
      SetBool::Response::SharedPtr response)
    {
      gate->mark_entered();
      gate->wait_until_released();
      response->success = true;
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  port.begin_goal();

  auto future = std::async(
    std::launch::async,
    [&]() {return port.attach(attach_request(), context);});
  ASSERT_TRUE(gate->wait_until_entered());
  context.cancel();
  port.request_stop();
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const auto result = future.get();
  gate->release();

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.attach_count, 0U);
  EXPECT_GT(scene.stop_count, 0U);
  EXPECT_FALSE(port.cached_readiness().ready);
}

TEST_F(IsaacPhysicalScenePortTest, StopAtPostDispatchGateCannotReturnCanceled)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("gate_stop_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("gate_stop_server"));
  const auto service_name = "/" + unique_name("gate_stop_pick_cube");
  FakeScenePort scene;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [](
      const SetBool::Request::SharedPtr,
      SetBool::Response::SharedPtr response)
    {
      response->success = true;
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  StopAtDispatchGateContext context;
  port.begin_goal();

  const auto result = port.attach(attach_request(), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.attach_count, 0U);
  EXPECT_FALSE(port.cached_readiness().ready);
}

TEST_F(IsaacPhysicalScenePortTest, DetachAlsoRequiresPhysicalConfirmationFirst)
{
  auto client_node = std::make_shared<rclcpp::Node>(unique_name("detach_client"));
  auto server_node = std::make_shared<rclcpp::Node>(unique_name("detach_server"));
  const auto service_name = "/" + unique_name("detach_pick_cube");
  FakeScenePort scene;
  scene.barrier_confirmed.store(true, std::memory_order_release);
  scene.fail_detach = true;
  auto service = server_node->create_service<SetBool>(
    service_name,
    [&scene](
      const SetBool::Request::SharedPtr request,
      SetBool::Response::SharedPtr response)
    {
      scene.barrier_confirmed.store(!request->data, std::memory_order_release);
      response->success = true;
    });
  ExecutorThread executor({client_node, server_node});
  IsaacPhysicalScenePort port(
    *client_node, scene, {service_name, "cube"},
    []() {return PickLiftPlaceReadiness{true, "raw scene ready"};});
  TestContext context;
  port.begin_goal();

  const auto result = port.detach(detach_request(), context);

  EXPECT_EQ(result.outcome(), CoordinatedSceneOutcome::kStateUncertain);
  EXPECT_EQ(scene.detach_count, 1U);
  EXPECT_TRUE(scene.detach_observed_barrier);
}

TEST(IsaacPhysicalScenePortConfigTest, AllowsDisabledServiceAndRejectsMalformedBindings)
{
  EXPECT_NO_THROW(
    franka_motion::workflow_v2::validateIsaacPhysicalScenePortConfig({"", "cube"}));
  EXPECT_THROW(
    franka_motion::workflow_v2::validateIsaacPhysicalScenePortConfig(
      {"relative/service", "cube"}),
    std::invalid_argument);
  EXPECT_THROW(
    franka_motion::workflow_v2::validateIsaacPhysicalScenePortConfig(
      {"/valid_service", ""}),
    std::invalid_argument);
}
