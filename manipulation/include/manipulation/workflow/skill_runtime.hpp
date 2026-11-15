// Responsibility: Binds immutable skill metadata to shared resource and recording services.

#ifndef MANIPULATION__WORKFLOW__SKILL_RUNTIME_HPP_
#define MANIPULATION__WORKFLOW__SKILL_RUNTIME_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "manipulation/recording/episode_event_sink.hpp"
#include "manipulation/skill/capability_set.hpp"
#include "manipulation/skill/resource_registry.hpp"
#include "manipulation/skill/skill_descriptor.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation/world/world_state_snapshot.hpp"

namespace manipulation::workflow
{

/** Typed result of one nonblocking skill resource-admission attempt. */
struct SkillAcquireResult
{
  manipulation::skill::ResourceAcquireError error{
    manipulation::skill::ResourceAcquireError::kInvalidOwner};
  std::optional<manipulation::skill::ResourceRegistry::Lease> lease;
  std::string conflicting_resource_id;

  /** A read-only success intentionally has no lease. */
  [[nodiscard]] bool acquired() const noexcept;
  [[nodiscard]] bool read_only() const noexcept;
};

/**
 * Immutable-by-API runtime context shared by one class of skill executions.
 *
 * The factory validates deployment capabilities before the runtime becomes
 * usable. Resource acquisition and recording are deliberately nonblocking:
 * this class performs no retries, waits, ROS communication, controller
 * switching, or storage I/O.
 */
class SkillRuntime final
{
public:
  /**
   * Build an admitted runtime for one immutable descriptor.
   *
   * Throws `std::invalid_argument` when capabilities are missing or a skill
   * with resource claims has no shared registry. A null sink is replaced with
   * an internal NullEpisodeEventSink. Read-only skills may omit the registry.
   */
  [[nodiscard]] static SkillRuntime create(
    manipulation::skill::SkillDescriptor descriptor,
    manipulation::skill::CapabilitySet available_capabilities,
    std::shared_ptr<manipulation::skill::ResourceRegistry> resource_registry,
    std::shared_ptr<manipulation::recording::EpisodeEventSink> event_sink = nullptr);

  [[nodiscard]] const manipulation::skill::SkillDescriptor & descriptor() const noexcept;
  [[nodiscard]] const manipulation::skill::CapabilitySet & available_capabilities()
  const noexcept;

  /**
   * Try to own all descriptor resources atomically without waiting.
   *
   * Empty claims represent a read-only skill and return success without a
   * lease. Otherwise the result and ownership semantics come directly from
   * the shared process-local ResourceRegistry.
   */
  [[nodiscard]] SkillAcquireResult try_acquire(std::string owner_id) const;

  /**
   * Construct and enqueue one event without allowing recording to affect work.
   *
   * Validation, allocation, or sink backpressure returns false. Every
   * exception raised while constructing the event is contained by this
   * noexcept boundary.
   */
  [[nodiscard]] bool try_record(
    manipulation::recording::EpisodeLifecycleMarker marker,
    std::string_view episode_id,
    std::chrono::steady_clock::duration elapsed,
    const manipulation::task::TaskStateSnapshot & state,
    const std::optional<manipulation::task::TaskOutcome> & outcome = std::nullopt,
    std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state = nullptr)
  const noexcept;

private:
  SkillRuntime(
    manipulation::skill::SkillDescriptor descriptor,
    manipulation::skill::CapabilitySet available_capabilities,
    std::shared_ptr<manipulation::skill::ResourceRegistry> resource_registry,
    std::shared_ptr<manipulation::recording::EpisodeEventSink> event_sink);

  manipulation::skill::SkillDescriptor descriptor_;
  manipulation::skill::CapabilitySet available_capabilities_;
  std::shared_ptr<manipulation::skill::ResourceRegistry> resource_registry_;
  std::shared_ptr<manipulation::recording::EpisodeEventSink> event_sink_;
};

}  // namespace manipulation::workflow

#endif  // MANIPULATION__WORKFLOW__SKILL_RUNTIME_HPP_
