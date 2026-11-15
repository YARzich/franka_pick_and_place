// Responsibility: Verifies episode-event invariants and the nonblocking null sink contract.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "manipulation/recording/episode_event_sink.hpp"

namespace
{

using manipulation::recording::EpisodeEvent;
using manipulation::recording::EpisodeLifecycleMarker;
using manipulation::skill::PresetIdentity;
using manipulation::task::ObjectDisposition;
using manipulation::task::TaskErrorCode;
using manipulation::task::TaskOutcome;
using manipulation::task::TaskPhase;
using manipulation::task::TaskStateSnapshot;
using manipulation::world::WorldStateProvenance;
using manipulation::world::WorldStateSnapshot;
using namespace std::chrono_literals;

PresetIdentity preset()
{
  return PresetIdentity::from_digest(
    "tabletop/cube_transfer", 3U, std::string(64U, 'a'));
}

TaskStateSnapshot state(
  const TaskPhase phase = TaskPhase::kPlanning,
  const std::chrono::steady_clock::duration elapsed = 1500ms)
{
  TaskStateSnapshot result;
  result.phase = phase;
  result.progress = phase == TaskPhase::kCompleted ? 1.0F : 0.25F;
  result.elapsed = elapsed;
  result.detail = "recorded state";
  return result;
}

std::shared_ptr<const WorldStateSnapshot> world_state(std::string token)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 17;
  return std::make_shared<const WorldStateSnapshot>(WorldStateSnapshot::create(
      WorldStateProvenance{"test/world", "test/session", 42U, stamp},
      std::move(token), {}));
}

EpisodeEvent event()
{
  return EpisodeEvent::create(
    EpisodeLifecycleMarker::kStateChanged,
    "123e4567-e89b-12d3-a456-426614174000",
    "transfer_object",
    preset(),
    1500ms,
    state(),
    std::nullopt,
    world_state("world/session-17/42"));
}

TEST(EpisodeEvent, OwnsStableBackendNeutralContext)
{
  const auto recorded = event();

  EXPECT_EQ(recorded.marker(), EpisodeLifecycleMarker::kStateChanged);
  EXPECT_EQ(recorded.episode_id(), "123e4567-e89b-12d3-a456-426614174000");
  EXPECT_EQ(recorded.skill_type(), "transfer_object");
  EXPECT_EQ(recorded.preset(), preset());
  EXPECT_EQ(recorded.elapsed(), 1500ms);
  EXPECT_EQ(recorded.state().phase, TaskPhase::kPlanning);
  EXPECT_FALSE(recorded.outcome());
  ASSERT_TRUE(recorded.world_state());
  ASSERT_TRUE(recorded.world_state_token());
  EXPECT_EQ(*recorded.world_state_token(), "world/session-17/42");
}

TEST(EpisodeEvent, ClassifiesTerminalMarkers)
{
  EXPECT_FALSE(manipulation::recording::is_terminal(EpisodeLifecycleMarker::kAccepted));
  EXPECT_FALSE(manipulation::recording::is_terminal(EpisodeLifecycleMarker::kRecoveryStarted));
  EXPECT_TRUE(manipulation::recording::is_terminal(EpisodeLifecycleMarker::kSucceeded));
  EXPECT_TRUE(manipulation::recording::is_terminal(EpisodeLifecycleMarker::kCanceled));
  EXPECT_TRUE(manipulation::recording::is_terminal(EpisodeLifecycleMarker::kFailed));
}

TEST(EpisodeEvent, AcceptsMatchingTerminalOutcomes)
{
  const auto succeeded = EpisodeEvent::create(
    EpisodeLifecycleMarker::kSucceeded, "episode-success", "transfer_object",
    preset(), 2s, state(TaskPhase::kCompleted, 2s),
    TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget),
    world_state("world-success"));
  ASSERT_TRUE(succeeded.outcome());
  EXPECT_TRUE(succeeded.outcome()->succeeded());

  const auto canceled = EpisodeEvent::create(
    EpisodeLifecycleMarker::kCanceled, "episode-canceled", "transfer_object",
    preset(), 2s, state(TaskPhase::kCanceled, 2s),
    TaskOutcome::make_failure(
      TaskErrorCode::kCanceled, ObjectDisposition::kObjectAtSource),
    world_state("world-canceled"));
  ASSERT_TRUE(canceled.outcome());
  EXPECT_EQ(canceled.outcome()->error_code(), TaskErrorCode::kCanceled);

  EXPECT_NO_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kFailed, "episode-failed", "transfer_object",
      preset(), 2s, state(TaskPhase::kFailed, 2s),
      TaskOutcome::make_failure(
        TaskErrorCode::kExecutionFailed, ObjectDisposition::kSceneUncertain),
      world_state("world-failed")));
}

TEST(EpisodeEvent, RequiresWorldTokenOnlyWhenSnapshotIsSelected)
{
  const auto started = EpisodeEvent::create(
    EpisodeLifecycleMarker::kStarted, "episode-started", "transfer_object",
    preset(), 1500ms, state(), std::nullopt, nullptr);
  EXPECT_FALSE(started.world_state_token());

  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kWorldStateSelected, "episode-select", "transfer_object",
      preset(), 1500ms, state(), std::nullopt, nullptr),
    std::invalid_argument);

  const auto selected = EpisodeEvent::create(
    EpisodeLifecycleMarker::kWorldStateSelected, "episode-select", "transfer_object",
    preset(), 1500ms, state(), std::nullopt, world_state("world-selected"));
  ASSERT_TRUE(selected.world_state());
  ASSERT_TRUE(selected.world_state_token());
  EXPECT_EQ(*selected.world_state_token(), "world-selected");
}

TEST(EpisodeEvent, NonterminalLifecycleMarkersAcceptNonterminalTaskPhases)
{
  EXPECT_NO_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kPaused, "episode-paused", "transfer_object",
      preset(), 1500ms, state(TaskPhase::kWaitingForResources), std::nullopt,
      nullptr));
  EXPECT_NO_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kResumed, "episode-resumed", "transfer_object",
      preset(), 1500ms, state(TaskPhase::kTransferring), std::nullopt,
      nullptr));
  EXPECT_NO_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kRecoveryStarted, "episode-recovery", "transfer_object",
      preset(), 1500ms, state(TaskPhase::kRecovering), std::nullopt,
      nullptr));
}

TEST(EpisodeEvent, RejectsMalformedStableIdentityFields)
{
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStarted, "", "transfer_object", preset(),
      1500ms, state(), std::nullopt, world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStarted, "episode 1", "transfer_object", preset(),
      1500ms, state(), std::nullopt, world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStarted, "episode-1", "Bad Skill", preset(),
      1500ms, state(), std::nullopt, world_state("world-42")),
    std::invalid_argument);
}

TEST(EpisodeEvent, RejectsInconsistentElapsedTimeAndProgress)
{
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStateChanged, "episode-1", "transfer_object", preset(),
      -1ms, state(TaskPhase::kPlanning, -1ms), std::nullopt, world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStateChanged, "episode-1", "transfer_object", preset(),
      1s, state(TaskPhase::kPlanning, 2s), std::nullopt, world_state("world-42")),
    std::invalid_argument);

  auto invalid_progress = state();
  invalid_progress.progress = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStateChanged, "episode-1", "transfer_object", preset(),
      1500ms, std::move(invalid_progress), std::nullopt, world_state("world-42")),
    std::invalid_argument);
}

TEST(EpisodeEvent, RejectsOutcomeAndLifecycleMismatches)
{
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kSucceeded, "episode-1", "transfer_object", preset(),
      2s, state(TaskPhase::kCompleted, 2s), std::nullopt, world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kStarted, "episode-1", "transfer_object", preset(),
      1500ms, state(),
      TaskOutcome::make_success(ObjectDisposition::kObjectAtTarget),
      world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kSucceeded, "episode-1", "transfer_object", preset(),
      2s, state(TaskPhase::kCompleted, 2s),
      TaskOutcome::make_failure(
        TaskErrorCode::kExecutionFailed, ObjectDisposition::kUnknown),
      world_state("world-42")),
    std::invalid_argument);
  EXPECT_THROW(
    EpisodeEvent::create(
      EpisodeLifecycleMarker::kFailed, "episode-1", "transfer_object", preset(),
      2s, state(TaskPhase::kPlanning, 2s),
      TaskOutcome::make_failure(
        TaskErrorCode::kExecutionFailed, ObjectDisposition::kUnknown),
      world_state("world-42")),
    std::invalid_argument);
}

TEST(EpisodeEventSink, NullSinkIsNoexceptAndNeverBackpressures)
{
  manipulation::recording::NullEpisodeEventSink sink;
  static_assert(noexcept(sink.try_enqueue(std::declval<const EpisodeEvent &>())));

  EXPECT_TRUE(sink.try_enqueue(event()));
}

}  // namespace
