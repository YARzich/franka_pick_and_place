// Responsibility: Defines an immutable, backend-neutral snapshot of observed rigid objects.

#ifndef MANIPULATION__WORLD__WORLD_STATE_SNAPSHOT_HPP_
#define MANIPULATION__WORLD__WORLD_STATE_SNAPSHOT_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace manipulation::world
{

/** Identifies where and when one world snapshot was produced. */
struct WorldStateProvenance
{
  std::string source;
  std::string session_id;
  std::uint64_t revision{0U};
  builtin_interfaces::msg::Time capture_stamp;
};

/** Pose and optional normalized confidence for one rigid object observation. */
struct RigidObjectState
{
  std::string object_id;
  geometry_msgs::msg::PoseStamped pose;
  std::optional<double> confidence;
};

/**
 * Immutable value representing one coherent view of the manipulation world.
 *
 * Construction validates provenance, the opaque snapshot token, every stamped
 * rigid pose, and optional confidence values. Objects are stored in ascending
 * `object_id` order and duplicate IDs are rejected. The snapshot deliberately
 * carries no planner, robot, object-shape, or environment-specific types.
 */
class WorldStateSnapshot final
{
public:
  /**
   * Validate and own one snapshot.
   *
   * Throws `std::invalid_argument` for invalid provenance, token, object ID,
   * pose, confidence, or duplicate object identity. An empty object set is a
   * valid observation of an empty or currently unknown world.
   */
  [[nodiscard]] static WorldStateSnapshot create(
    WorldStateProvenance provenance,
    std::string token,
    std::vector<RigidObjectState> objects);

  [[nodiscard]] const WorldStateProvenance & provenance() const noexcept;
  [[nodiscard]] const std::string & token() const noexcept;
  [[nodiscard]] const std::vector<RigidObjectState> & objects() const noexcept;

  /** Return the sorted object entry, or `nullptr` when the ID is absent. */
  [[nodiscard]] const RigidObjectState * find_object(
    std::string_view object_id) const noexcept;

private:
  WorldStateSnapshot(
    WorldStateProvenance provenance,
    std::string token,
    std::vector<RigidObjectState> objects);

  WorldStateProvenance provenance_;
  std::string token_;
  std::vector<RigidObjectState> objects_;
};

}  // namespace manipulation::world

#endif  // MANIPULATION__WORLD__WORLD_STATE_SNAPSHOT_HPP_
