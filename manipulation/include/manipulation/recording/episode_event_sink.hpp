// Responsibility: Defines backend-neutral manipulation episode events and nonblocking sinks.

#ifndef MANIPULATION__RECORDING__EPISODE_EVENT_SINK_HPP_
#define MANIPULATION__RECORDING__EPISODE_EVENT_SINK_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "manipulation/skill/preset_identity.hpp"
#include "manipulation/task/task_status.hpp"
#include "manipulation/world/world_state_snapshot.hpp"

namespace manipulation::recording
{

/** Stable lifecycle boundary represented by one episode event. */
enum class EpisodeLifecycleMarker : std::uint8_t
{
  kAccepted,
  kStarted,
  kWorldStateSelected,
  kStateChanged,
  kPaused,
  kResumed,
  kRecoveryStarted,
  kSucceeded,
  kCanceled,
  kFailed,
};

/** Return true when a lifecycle marker closes the episode. */
[[nodiscard]] bool is_terminal(EpisodeLifecycleMarker marker) noexcept;

/**
 * Immutable event emitted by a skill runtime without selecting a storage backend.
 *
 * `elapsed` and the embedded task snapshot use the same steady-clock duration.
 * Terminal markers require a matching terminal outcome; non-terminal markers
 * cannot carry one. The optional shared world snapshot preserves the exact
 * immutable planning input without copying it for every event. It is required
 * by `kWorldStateSelected`, while Accepted and Started can be recorded honestly
 * before a snapshot has been chosen.
 */
class EpisodeEvent final
{
public:
  /** Validate and own one event, throwing `std::invalid_argument` on mismatch. */
  [[nodiscard]] static EpisodeEvent create(
    EpisodeLifecycleMarker marker,
    std::string episode_id,
    std::string skill_type,
    manipulation::skill::PresetIdentity preset,
    std::chrono::steady_clock::duration elapsed,
    manipulation::task::TaskStateSnapshot state,
    std::optional<manipulation::task::TaskOutcome> outcome,
    std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state);

  /** As above, also identifying the exact process/deployment configuration. */
  [[nodiscard]] static EpisodeEvent create(
    EpisodeLifecycleMarker marker,
    std::string episode_id,
    std::string skill_type,
    manipulation::skill::PresetIdentity preset,
    std::optional<manipulation::skill::PresetIdentity> deployment_identity,
    std::chrono::steady_clock::duration elapsed,
    manipulation::task::TaskStateSnapshot state,
    std::optional<manipulation::task::TaskOutcome> outcome,
    std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state);

  [[nodiscard]] EpisodeLifecycleMarker marker() const noexcept;
  [[nodiscard]] const std::string & episode_id() const noexcept;
  [[nodiscard]] const std::string & skill_type() const noexcept;
  [[nodiscard]] const manipulation::skill::PresetIdentity & preset() const noexcept;
  [[nodiscard]] const std::optional<manipulation::skill::PresetIdentity> &
  deployment_identity() const noexcept;
  [[nodiscard]] std::chrono::steady_clock::duration elapsed() const noexcept;
  [[nodiscard]] const manipulation::task::TaskStateSnapshot & state() const noexcept;
  [[nodiscard]] const std::optional<manipulation::task::TaskOutcome> & outcome() const noexcept;
  [[nodiscard]] const std::shared_ptr<const manipulation::world::WorldStateSnapshot> &
  world_state() const noexcept;
  [[nodiscard]] const std::optional<std::string> & world_state_token() const noexcept;

private:
  EpisodeEvent(
    EpisodeLifecycleMarker marker,
    std::string episode_id,
    std::string skill_type,
    manipulation::skill::PresetIdentity preset,
    std::optional<manipulation::skill::PresetIdentity> deployment_identity,
    std::chrono::steady_clock::duration elapsed,
    manipulation::task::TaskStateSnapshot state,
    std::optional<manipulation::task::TaskOutcome> outcome,
    std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state);

  EpisodeLifecycleMarker marker_;
  std::string episode_id_;
  std::string skill_type_;
  manipulation::skill::PresetIdentity preset_;
  std::optional<manipulation::skill::PresetIdentity> deployment_identity_;
  std::chrono::steady_clock::duration elapsed_;
  manipulation::task::TaskStateSnapshot state_;
  std::optional<manipulation::task::TaskOutcome> outcome_;
  std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state_;
  std::optional<std::string> world_state_token_;
};

/**
 * Nonblocking event handoff boundary.
 *
 * Implementations must return immediately, perform no storage or network I/O,
 * and return false when a bounded queue cannot accept the event. They must not
 * retain the reference after the call returns unless they first copy it into
 * owned bounded storage while honoring the noexcept contract.
 */
class EpisodeEventSink
{
public:
  virtual ~EpisodeEventSink();

  [[nodiscard]] virtual bool try_enqueue(const EpisodeEvent & event) noexcept = 0;
};

/** Explicitly discard recording events while reporting successful handoff. */
class NullEpisodeEventSink final : public EpisodeEventSink
{
public:
  [[nodiscard]] bool try_enqueue(const EpisodeEvent & event) noexcept override;
};

}  // namespace manipulation::recording

#endif  // MANIPULATION__RECORDING__EPISODE_EVENT_SINK_HPP_
