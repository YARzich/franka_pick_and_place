// Responsibility: Defines an immutable normalized set of logical skill capabilities.

#ifndef MANIPULATION__SKILL__CAPABILITY_SET_HPP_
#define MANIPULATION__SKILL__CAPABILITY_SET_HPP_

#include <cstddef>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace manipulation::skill
{

/**
 * Sorted unique collection of capabilities available to or required by a skill.
 *
 * Construction rejects malformed logical IDs. Duplicate valid IDs are folded
 * into one entry, giving deterministic comparison, diagnostics, and recording.
 * The value exposes no mutating operation after construction.
 */
class CapabilitySet final
{
public:
  CapabilitySet() = default;
  explicit CapabilitySet(std::vector<std::string> capabilities);
  CapabilitySet(std::initializer_list<std::string> capabilities);

  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] bool contains(std::string_view capability) const noexcept;

  /** Return true when this available set contains every required capability. */
  [[nodiscard]] bool satisfies(const CapabilitySet & required) const noexcept;

  /** Return required capabilities absent from this set in canonical sorted order. */
  [[nodiscard]] std::vector<std::string> missing(
    const CapabilitySet & required) const;

  [[nodiscard]] const std::vector<std::string> & values() const noexcept;

  [[nodiscard]] bool operator==(const CapabilitySet & other) const noexcept;
  [[nodiscard]] bool operator!=(const CapabilitySet & other) const noexcept;

private:
  std::vector<std::string> capabilities_;
};

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__CAPABILITY_SET_HPP_
