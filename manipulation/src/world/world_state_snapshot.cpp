// Responsibility: Validates and canonicalizes backend-neutral world-state snapshots.

#include "manipulation/world/world_state_snapshot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "manipulation/validation/rigid_pose.hpp"

namespace manipulation::world
{
namespace
{

constexpr std::size_t kMaximumIdentityLength = 256U;

bool is_bounded_visible_token(const std::string_view value) noexcept
{
  if (value.empty() || value.size() > kMaximumIdentityLength) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const char character) {
             const auto byte = static_cast<unsigned char>(character);
             return byte >= 0x21U && byte <= 0x7eU;
  });
}

bool is_valid_confidence(const std::optional<double> confidence) noexcept
{
  return !confidence ||
         (std::isfinite(*confidence) && *confidence >= 0.0 && *confidence <= 1.0);
}

}  // namespace

WorldStateSnapshot WorldStateSnapshot::create(
  WorldStateProvenance provenance,
  std::string token,
  std::vector<RigidObjectState> objects)
{
  if (!is_bounded_visible_token(provenance.source)) {
    throw std::invalid_argument("World-state source must be a bounded visible token");
  }
  if (!is_bounded_visible_token(provenance.session_id)) {
    throw std::invalid_argument("World-state session ID must be a bounded visible token");
  }
  if (!manipulation::validation::is_normalized_nonzero_stamp(
      provenance.capture_stamp))
  {
    throw std::invalid_argument("World-state capture stamp must be normalized and non-zero");
  }
  if (!is_bounded_visible_token(token)) {
    throw std::invalid_argument("World-state token must be a bounded visible token");
  }

  for (const auto & object : objects) {
    if (!is_bounded_visible_token(object.object_id)) {
      throw std::invalid_argument("Rigid object ID must be a bounded visible token");
    }
    if (!manipulation::validation::is_valid_pose_stamped(object.pose)) {
      throw std::invalid_argument("Rigid object pose must be a valid stamped pose");
    }
    if (!is_valid_confidence(object.confidence)) {
      throw std::invalid_argument("Rigid object confidence must be finite and in [0, 1]");
    }
  }

  std::sort(
    objects.begin(), objects.end(),
    [](const RigidObjectState & left, const RigidObjectState & right) {
      return left.object_id < right.object_id;
    });
  const auto duplicate = std::adjacent_find(
    objects.cbegin(), objects.cend(),
    [](const RigidObjectState & left, const RigidObjectState & right) {
      return left.object_id == right.object_id;
    });
  if (duplicate != objects.cend()) {
    throw std::invalid_argument("World-state object IDs must be unique");
  }

  return WorldStateSnapshot(
    std::move(provenance), std::move(token), std::move(objects));
}

const WorldStateProvenance & WorldStateSnapshot::provenance() const noexcept
{
  return provenance_;
}

const std::string & WorldStateSnapshot::token() const noexcept
{
  return token_;
}

const std::vector<RigidObjectState> & WorldStateSnapshot::objects() const noexcept
{
  return objects_;
}

const RigidObjectState * WorldStateSnapshot::find_object(
  const std::string_view object_id) const noexcept
{
  const auto object = std::lower_bound(
    objects_.cbegin(), objects_.cend(), object_id,
    [](const RigidObjectState & candidate, const std::string_view requested_id) {
      return std::string_view{candidate.object_id} < requested_id;
    });
  return object != objects_.cend() && std::string_view{object->object_id} == object_id ?
         &*object : nullptr;
}

WorldStateSnapshot::WorldStateSnapshot(
  WorldStateProvenance provenance,
  std::string token,
  std::vector<RigidObjectState> objects)
: provenance_(std::move(provenance)),
  token_(std::move(token)),
  objects_(std::move(objects))
{
}

}  // namespace manipulation::world
