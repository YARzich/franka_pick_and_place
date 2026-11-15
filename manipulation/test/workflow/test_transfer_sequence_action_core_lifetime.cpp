// Responsibility: Verifies pre-worker snapshots and bounded core destruction.

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "transfer_sequence_action_core_test_support.hpp"

namespace manipulation::workflow::detail
{
namespace
{

using namespace std::chrono_literals;
using test::TransferSequenceActionCoreTest;
using test::accepted_admission;
using test::descriptor;
using test::goal_id;
using test::successful_execution;
using test::transport_port;

TEST_F(TransferSequenceActionCoreTest, SnapshotHasValidatedStepCountBeforeWorkerStarts)
{
  TransferSequenceLifecycleConfig config;
  config.accepted_start_delay = 250ms;
  auto core = make_core("pre_worker_snapshot", config);
  const auto id = goal_id(101U);
  auto probe = std::make_shared<test::TransportProbe>();
  auto goal = descriptor(3U);
  goal.requested_timeout = 500ms;

  ASSERT_EQ(
    core->handle_goal(
      id, goal, accepted_admission,
      [](TransferSequenceTaskContext &) {return successful_execution(3U);}),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));

  const auto snapshot = core->snapshot();
  EXPECT_TRUE(snapshot.goal_reserved);
  EXPECT_TRUE(snapshot.active);
  EXPECT_EQ(snapshot.sequence_step_index, 0U);
  EXPECT_EQ(snapshot.sequence_step_count, 3U);
  EXPECT_EQ(snapshot.phase, manipulation::task::TaskPhase::kValidating);
  EXPECT_FALSE(snapshot.backend_invoked);
  EXPECT_EQ(snapshot.worker_state, TaskWorkerState::kIdle);
}

TEST_F(TransferSequenceActionCoreTest, DestructionStopsAndJoinsBlockedBackend)
{
  std::atomic_bool backend_started{false};
  std::atomic_bool release_backend{false};
  std::atomic_uint stop_count{0U};
  auto core = make_core(
    "destruction_join", {},
    [&release_backend, &stop_count](TaskStopCause) {
      ++stop_count;
      release_backend.store(true);
    });
  const auto id = goal_id(121U);
  auto probe = std::make_shared<test::TransportProbe>();

  ASSERT_EQ(
    core->handle_goal(
      id, descriptor(), accepted_admission,
      [&backend_started, &release_backend](TransferSequenceTaskContext &) {
        backend_started.store(true);
        while (!release_backend.load()) {
          std::this_thread::sleep_for(1ms);
        }
        return successful_execution();
      }),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  core->handle_accepted(transport_port(id, probe));
  ASSERT_TRUE(spin_until([&backend_started]() {return backend_started.load();}));

  remove_node_from_executor();
  const auto started_at = std::chrono::steady_clock::now();
  core.reset();
  const auto elapsed = std::chrono::steady_clock::now() - started_at;

  EXPECT_EQ(stop_count.load(), 1U);
  EXPECT_TRUE(release_backend.load());
  EXPECT_LT(elapsed, 500ms);
}

}  // namespace
}  // namespace manipulation::workflow::detail
