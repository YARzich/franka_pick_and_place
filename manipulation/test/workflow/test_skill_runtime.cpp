// Responsibility: Verifies skill admission, shared resource arbitration, and safe recording.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "manipulation/recording/episode_event_sink.hpp"
#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/preset_identity.hpp"
#include "manipulation/skill/resource_claim.hpp"
#include "manipulation/skill/resource_registry.hpp"
#include "manipulation/skill/skill_descriptor.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation/workflow/skill_runtime.hpp"

namespace
{

using manipulation::recording::EpisodeEvent;
using manipulation::recording::EpisodeEventSink;
using manipulation::recording::EpisodeLifecycleMarker;
using manipulation::skill::CapabilitySet;
using manipulation::skill::PresetIdentity;
using manipulation::skill::ResourceAcquireError;
using manipulation::skill::ResourceClaim;
using manipulation::skill::ResourceRegistry;
using manipulation::skill::SkillDescriptor;
using manipulation::task::TaskPhase;
using manipulation::task::TaskStateSnapshot;
using manipulation::workflow::SkillRuntime;
using manipulation::world::WorldStateProvenance;
using manipulation::world::WorldStateSnapshot;
using namespace std::chrono_literals;

PresetIdentity preset()
{
  return PresetIdentity::from_digest(
    "tabletop/transfer", 1U, std::string(64U, 'b'));
}

SkillDescriptor descriptor(
  std::string skill_type,
  CapabilitySet required,
  std::vector<ResourceClaim> claims)
{
  return SkillDescriptor::create(
    std::move(skill_type), preset(), std::move(required), std::move(claims));
}

TaskStateSnapshot planning_state()
{
  TaskStateSnapshot state;
  state.phase = TaskPhase::kPlanning;
  state.progress = 0.25F;
  state.elapsed = 1s;
  return state;
}

std::shared_ptr<const WorldStateSnapshot> world_state(std::string token)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 4;
  return std::make_shared<const WorldStateSnapshot>(WorldStateSnapshot::create(
      WorldStateProvenance{"test/world", "test/session", 4U, stamp},
      std::move(token), {}));
}

class CaptureSink final : public EpisodeEventSink
{
public:
  bool try_enqueue(const EpisodeEvent & event) noexcept override
  {
    ++calls;
    try {
      captured = event;
      return true;
    } catch (...) {
      return false;
    }
  }

  std::size_t calls{0U};
  std::optional<EpisodeEvent> captured;
};

class BackpressureSink final : public EpisodeEventSink
{
public:
  bool try_enqueue(const EpisodeEvent & event) noexcept override
  {
    static_cast<void>(event);
    ++calls;
    return false;
  }

  std::size_t calls{0U};
};

TEST(SkillRuntime, RejectsDeploymentWithMissingCapabilities)
{
  const auto skill = descriptor(
    "transfer_object",
    CapabilitySet{"motion/global", "gripper/position"},
    {});

  EXPECT_THROW(
    SkillRuntime::create(
      skill, CapabilitySet{"motion/global"}, nullptr),
    std::invalid_argument);
}

TEST(SkillRuntime, RequiresRegistryOnlyForExclusiveClaims)
{
  EXPECT_NO_THROW(
    SkillRuntime::create(
      descriptor("inspect_object", CapabilitySet{}, {}), CapabilitySet{}, nullptr));
  EXPECT_THROW(
    SkillRuntime::create(
      descriptor(
        "transfer_object", CapabilitySet{}, {ResourceClaim{"arm/main"}}),
      CapabilitySet{}, nullptr),
    std::invalid_argument);
}

TEST(SkillRuntime, ReadOnlySkillSucceedsWithoutArtificialLease)
{
  const auto runtime = SkillRuntime::create(
    descriptor("inspect_object", CapabilitySet{"perception/pose"}, {}),
    CapabilitySet{"perception/pose"}, nullptr);

  auto acquired = runtime.try_acquire("inspector/main");
  EXPECT_TRUE(acquired.acquired());
  EXPECT_TRUE(acquired.read_only());
  EXPECT_FALSE(acquired.lease);
  EXPECT_EQ(acquired.error, ResourceAcquireError::kNone);

  auto invalid_owner = runtime.try_acquire("Bad Owner");
  EXPECT_FALSE(invalid_owner.acquired());
  EXPECT_EQ(invalid_owner.error, ResourceAcquireError::kInvalidOwner);
}

TEST(SkillRuntime, SharedRegistryRejectsConflictsWithoutPartialAcquisition)
{
  const auto registry = std::make_shared<ResourceRegistry>();
  const auto first_runtime = SkillRuntime::create(
    descriptor(
      "transfer_object", CapabilitySet{},
      {ResourceClaim{"arm/main"}, ResourceClaim{"gripper/main"}}),
    CapabilitySet{}, registry);
  const auto second_runtime = SkillRuntime::create(
    descriptor(
      "move_mobile_manipulator", CapabilitySet{},
      {ResourceClaim{"arm/main"}, ResourceClaim{"base/main"}}),
    CapabilitySet{}, registry);

  auto first = first_runtime.try_acquire("task/first");
  ASSERT_TRUE(first.acquired());
  ASSERT_TRUE(first.lease);

  auto conflicting = second_runtime.try_acquire("task/second");
  EXPECT_FALSE(conflicting.acquired());
  EXPECT_EQ(conflicting.error, ResourceAcquireError::kConflict);
  EXPECT_EQ(conflicting.conflicting_resource_id, "arm/main");
  const auto during_conflict = registry->snapshot();
  ASSERT_EQ(during_conflict.active_ownerships.size(), 2U);
  EXPECT_EQ(during_conflict.active_ownerships[0].resource_id, "arm/main");
  EXPECT_EQ(during_conflict.active_ownerships[1].resource_id, "gripper/main");

  first.lease.reset();
  auto second = second_runtime.try_acquire("task/second");
  ASSERT_TRUE(second.acquired());
  ASSERT_TRUE(second.lease);
  EXPECT_EQ(second.lease->resource_ids(), std::vector<std::string>({
      "arm/main", "base/main"}));
}

TEST(SkillRuntime, RecordsDescriptorIdentityAndImmutableWorldState)
{
  const auto sink = std::make_shared<CaptureSink>();
  const auto runtime = SkillRuntime::create(
    descriptor("transfer_object", CapabilitySet{}, {}),
    CapabilitySet{}, nullptr, sink);
  const auto state = planning_state();

  EXPECT_TRUE(runtime.try_record(
      EpisodeLifecycleMarker::kWorldStateSelected,
      "episode-17", 1s, state, std::nullopt, world_state("world/session/4")));
  ASSERT_EQ(sink->calls, 1U);
  ASSERT_TRUE(sink->captured);
  EXPECT_EQ(sink->captured->skill_type(), "transfer_object");
  EXPECT_EQ(sink->captured->preset(), preset());
  ASSERT_TRUE(sink->captured->world_state());
  EXPECT_EQ(sink->captured->world_state()->provenance().revision, 4U);
  ASSERT_TRUE(sink->captured->world_state_token());
  EXPECT_EQ(*sink->captured->world_state_token(), "world/session/4");
}

TEST(SkillRuntime, RecordsDeploymentIdentityWhenDescriptorProvidesIt)
{
  const auto sink = std::make_shared<CaptureSink>();
  const auto deployment = PresetIdentity::from_canonical_content(
    "moveit/transfer_process", 1U, "timeout=30");
  const auto skill = SkillDescriptor::create(
    "transfer_object", preset(), CapabilitySet{}, {}, deployment);
  const auto runtime = SkillRuntime::create(skill, CapabilitySet{}, nullptr, sink);

  ASSERT_TRUE(runtime.try_record(
      EpisodeLifecycleMarker::kStarted,
      "episode-18", 1s, planning_state()));
  ASSERT_TRUE(sink->captured);
  ASSERT_TRUE(sink->captured->deployment_identity());
  EXPECT_EQ(*sink->captured->deployment_identity(), deployment);
}

TEST(SkillRuntime, ReportsBackpressureWithoutAffectingCaller)
{
  const auto sink = std::make_shared<BackpressureSink>();
  const auto runtime = SkillRuntime::create(
    descriptor("inspect_object", CapabilitySet{}, {}),
    CapabilitySet{}, nullptr, sink);

  EXPECT_FALSE(runtime.try_record(
      EpisodeLifecycleMarker::kStateChanged,
      "episode-17", 1s, planning_state()));
  EXPECT_EQ(sink->calls, 1U);
}

TEST(SkillRuntime, InvalidEventNeverThrowsOrReachesSink)
{
  const auto sink = std::make_shared<CaptureSink>();
  const auto runtime = SkillRuntime::create(
    descriptor("inspect_object", CapabilitySet{}, {}),
    CapabilitySet{}, nullptr, sink);
  auto inconsistent = planning_state();
  inconsistent.elapsed = 2s;

  bool recorded = true;
  EXPECT_NO_THROW(
    recorded = runtime.try_record(
      EpisodeLifecycleMarker::kStateChanged,
      "episode-17", 1s, inconsistent));
  EXPECT_FALSE(recorded);
  EXPECT_EQ(sink->calls, 0U);

  EXPECT_NO_THROW(
    recorded = runtime.try_record(
      EpisodeLifecycleMarker::kWorldStateSelected,
      "episode-17", 1s, planning_state()));
  EXPECT_FALSE(recorded);
  EXPECT_EQ(sink->calls, 0U);
}

TEST(SkillRuntime, NullSinkAcceptsValidEvents)
{
  const auto runtime = SkillRuntime::create(
    descriptor("inspect_object", CapabilitySet{}, {}),
    CapabilitySet{}, nullptr);

  EXPECT_TRUE(runtime.try_record(
      EpisodeLifecycleMarker::kStarted,
      "episode-17", 1s, planning_state()));
}

}  // namespace
