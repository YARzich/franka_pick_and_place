// Responsibility: Validates episode-event invariants and implements the no-op event sink.

#include "manipulation/recording/episode_event_sink.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::recording
{
namespace
{

constexpr std::size_t kMaximumTokenLength = 256U;

bool is_bounded_visible_token(const std::string_view value) noexcept
{
  if (value.empty() || value.size() > kMaximumTokenLength) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return byte >= 0x21U && byte <= 0x7eU;
  });
}

bool marker_matches_outcome(
  const EpisodeLifecycleMarker marker,
  const manipulation::task::TaskOutcome & outcome) noexcept
{
  using manipulation::task::TaskErrorCode;
  switch (marker) {
    case EpisodeLifecycleMarker::kSucceeded:
      return outcome.succeeded();
    case EpisodeLifecycleMarker::kCanceled:
      return !outcome.succeeded() && outcome.error_code() == TaskErrorCode::kCanceled;
    case EpisodeLifecycleMarker::kFailed:
      return !outcome.succeeded() && outcome.error_code() != TaskErrorCode::kCanceled;
    default:
      return false;
  }
}

bool marker_matches_phase(
  const EpisodeLifecycleMarker marker,
  const manipulation::task::TaskPhase phase) noexcept
{
  using manipulation::task::TaskPhase;
  switch (marker) {
    case EpisodeLifecycleMarker::kSucceeded:
      return phase == TaskPhase::kCompleted;
    case EpisodeLifecycleMarker::kCanceled:
      return phase == TaskPhase::kCanceled;
    case EpisodeLifecycleMarker::kFailed:
      return phase == TaskPhase::kFailed;
    default:
      return !manipulation::task::is_terminal(phase);
  }
}

}  // namespace

bool is_terminal(const EpisodeLifecycleMarker marker) noexcept
{
  return marker == EpisodeLifecycleMarker::kSucceeded ||
         marker == EpisodeLifecycleMarker::kCanceled ||
         marker == EpisodeLifecycleMarker::kFailed;
}

EpisodeEvent EpisodeEvent::create(
  const EpisodeLifecycleMarker marker,
  std::string episode_id,
  std::string skill_type,
  manipulation::skill::PresetIdentity preset,
  const std::chrono::steady_clock::duration elapsed,
  manipulation::task::TaskStateSnapshot state,
  std::optional<manipulation::task::TaskOutcome> outcome,
  std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state)
{
  return create(
    marker, std::move(episode_id), std::move(skill_type), std::move(preset),
    std::nullopt, elapsed, std::move(state), std::move(outcome),
    std::move(world_state));
}

EpisodeEvent EpisodeEvent::create(
  const EpisodeLifecycleMarker marker,
  std::string episode_id,
  std::string skill_type,
  manipulation::skill::PresetIdentity preset,
  std::optional<manipulation::skill::PresetIdentity> deployment_identity,
  const std::chrono::steady_clock::duration elapsed,
  manipulation::task::TaskStateSnapshot state,
  std::optional<manipulation::task::TaskOutcome> outcome,
  std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state)
{
  if (!is_bounded_visible_token(episode_id)) {
    throw std::invalid_argument("Episode ID must be a bounded visible token");
  }
  if (!manipulation::skill::is_canonical_logical_id(skill_type)) {
    throw std::invalid_argument("Skill type must be a canonical logical ID");
  }
  if (marker == EpisodeLifecycleMarker::kWorldStateSelected && !world_state) {
    throw std::invalid_argument("World-state selection event requires a snapshot");
  }
  if (elapsed < std::chrono::steady_clock::duration::zero() || state.elapsed != elapsed) {
    throw std::invalid_argument(
            "Episode elapsed time must be non-negative and match the task snapshot");
  }
  if (!std::isfinite(state.progress) || state.progress < 0.0F || state.progress > 1.0F) {
    throw std::invalid_argument("Episode task progress must be finite and in [0, 1]");
  }
  if (is_terminal(marker) != outcome.has_value()) {
    throw std::invalid_argument("Only terminal episode events must carry a task outcome");
  }
  if (outcome && !marker_matches_outcome(marker, *outcome)) {
    throw std::invalid_argument("Episode terminal marker does not match its task outcome");
  }
  if (!marker_matches_phase(marker, state.phase)) {
    throw std::invalid_argument("Episode lifecycle marker does not match the task phase");
  }

  return EpisodeEvent(
    marker, std::move(episode_id), std::move(skill_type), std::move(preset),
    std::move(deployment_identity), elapsed, std::move(state), std::move(outcome),
    std::move(world_state));
}

EpisodeLifecycleMarker EpisodeEvent::marker() const noexcept
{
  return marker_;
}

const std::string & EpisodeEvent::episode_id() const noexcept
{
  return episode_id_;
}

const std::string & EpisodeEvent::skill_type() const noexcept
{
  return skill_type_;
}

const manipulation::skill::PresetIdentity & EpisodeEvent::preset() const noexcept
{
  return preset_;
}

const std::optional<manipulation::skill::PresetIdentity> &
EpisodeEvent::deployment_identity() const noexcept
{
  return deployment_identity_;
}

std::chrono::steady_clock::duration EpisodeEvent::elapsed() const noexcept
{
  return elapsed_;
}

const manipulation::task::TaskStateSnapshot & EpisodeEvent::state() const noexcept
{
  return state_;
}

const std::optional<manipulation::task::TaskOutcome> & EpisodeEvent::outcome() const noexcept
{
  return outcome_;
}

const std::shared_ptr<const manipulation::world::WorldStateSnapshot> &
EpisodeEvent::world_state() const noexcept
{
  return world_state_;
}

const std::optional<std::string> & EpisodeEvent::world_state_token() const noexcept
{
  return world_state_token_;
}

EpisodeEvent::EpisodeEvent(
  const EpisodeLifecycleMarker marker,
  std::string episode_id,
  std::string skill_type,
  manipulation::skill::PresetIdentity preset,
  std::optional<manipulation::skill::PresetIdentity> deployment_identity,
  const std::chrono::steady_clock::duration elapsed,
  manipulation::task::TaskStateSnapshot state,
  std::optional<manipulation::task::TaskOutcome> outcome,
  std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state)
: marker_(marker),
  episode_id_(std::move(episode_id)),
  skill_type_(std::move(skill_type)),
  preset_(std::move(preset)),
  deployment_identity_(std::move(deployment_identity)),
  elapsed_(elapsed),
  state_(std::move(state)),
  outcome_(std::move(outcome)),
  world_state_(std::move(world_state)),
  world_state_token_(world_state_ ?
    std::optional<std::string>{world_state_->token()} : std::nullopt)
{
}

EpisodeEventSink::~EpisodeEventSink() = default;

bool NullEpisodeEventSink::try_enqueue(const EpisodeEvent & event) noexcept
{
  static_cast<void>(event);
  return true;
}

}  // namespace manipulation::recording
