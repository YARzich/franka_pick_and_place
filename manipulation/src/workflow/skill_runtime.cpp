// Responsibility: Implements capability admission, resource delegation, and safe event handoff.

#include "manipulation/workflow/skill_runtime.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "manipulation/skill/logical_id.hpp"

namespace manipulation::workflow
{
namespace
{

std::string describe_missing_capabilities(const std::vector<std::string> & missing)
{
  std::ostringstream description;
  for (std::size_t index = 0U; index < missing.size(); ++index) {
    if (index != 0U) {
      description << ", ";
    }
    description << missing[index];
  }
  return description.str();
}

}  // namespace

bool SkillAcquireResult::acquired() const noexcept
{
  return error == manipulation::skill::ResourceAcquireError::kNone;
}

bool SkillAcquireResult::read_only() const noexcept
{
  return acquired() && !lease;
}

SkillRuntime SkillRuntime::create(
  manipulation::skill::SkillDescriptor descriptor,
  manipulation::skill::CapabilitySet available_capabilities,
  std::shared_ptr<manipulation::skill::ResourceRegistry> resource_registry,
  std::shared_ptr<manipulation::recording::EpisodeEventSink> event_sink)
{
  const auto missing = available_capabilities.missing(
    descriptor.required_capabilities());
  if (!missing.empty()) {
    throw std::invalid_argument(
            "Skill runtime is missing required capabilities: " +
            describe_missing_capabilities(missing));
  }
  if (!descriptor.resource_claims().empty() && !resource_registry) {
    throw std::invalid_argument(
            "Skill runtime requires a shared resource registry for exclusive claims");
  }
  if (!event_sink) {
    event_sink = std::make_shared<manipulation::recording::NullEpisodeEventSink>();
  }
  return SkillRuntime(
    std::move(descriptor), std::move(available_capabilities),
    std::move(resource_registry), std::move(event_sink));
}

const manipulation::skill::SkillDescriptor & SkillRuntime::descriptor() const noexcept
{
  return descriptor_;
}

const manipulation::skill::CapabilitySet & SkillRuntime::available_capabilities()
const noexcept
{
  return available_capabilities_;
}

SkillAcquireResult SkillRuntime::try_acquire(std::string owner_id) const
{
  if (descriptor_.resource_claims().empty()) {
    if (!manipulation::skill::is_canonical_logical_id(owner_id)) {
      return {
        manipulation::skill::ResourceAcquireError::kInvalidOwner,
        std::nullopt,
        {}};
    }
    return {
      manipulation::skill::ResourceAcquireError::kNone,
      std::nullopt,
      {}};
  }

  auto acquired = resource_registry_->try_acquire(
    std::move(owner_id), descriptor_.resource_claims());
  return {
    acquired.error,
    std::move(acquired.lease),
    std::move(acquired.conflicting_resource_id)};
}

bool SkillRuntime::try_record(
  const manipulation::recording::EpisodeLifecycleMarker marker,
  const std::string_view episode_id,
  const std::chrono::steady_clock::duration elapsed,
  const manipulation::task::TaskStateSnapshot & state,
  const std::optional<manipulation::task::TaskOutcome> & outcome,
  std::shared_ptr<const manipulation::world::WorldStateSnapshot> world_state) const noexcept
{
  try {
    const auto event = manipulation::recording::EpisodeEvent::create(
      marker,
      std::string{episode_id},
      descriptor_.skill_type(),
      descriptor_.preset(),
      descriptor_.deployment_identity(),
      elapsed,
      state,
      outcome,
      std::move(world_state));
    return event_sink_->try_enqueue(event);
  } catch (...) {
    return false;
  }
}

SkillRuntime::SkillRuntime(
  manipulation::skill::SkillDescriptor descriptor,
  manipulation::skill::CapabilitySet available_capabilities,
  std::shared_ptr<manipulation::skill::ResourceRegistry> resource_registry,
  std::shared_ptr<manipulation::recording::EpisodeEventSink> event_sink)
: descriptor_(std::move(descriptor)),
  available_capabilities_(std::move(available_capabilities)),
  resource_registry_(std::move(resource_registry)),
  event_sink_(std::move(event_sink))
{
}

}  // namespace manipulation::workflow
