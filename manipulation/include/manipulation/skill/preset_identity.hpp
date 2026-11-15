// Responsibility: Identifies an immutable, versioned skill preset by its canonical content.

#ifndef MANIPULATION__SKILL__PRESET_IDENTITY_HPP_
#define MANIPULATION__SKILL__PRESET_IDENTITY_HPP_

#include <cstdint>
#include <string>
#include <string_view>

namespace manipulation::skill
{

/**
 * Stable identity of the exact configuration used by a skill execution.
 *
 * The human-readable ID names the preset family. The schema version describes
 * how its canonical content is interpreted. The SHA-256 digest distinguishes
 * two configurations that share the same name and schema.
 */
class PresetIdentity final
{
public:
  /** Build an identity and calculate its digest from deterministic content. */
  [[nodiscard]] static PresetIdentity from_canonical_content(
    std::string preset_id, std::uint32_t schema_version,
    std::string_view canonical_content);

  /** Reconstruct an identity from a previously calculated lowercase SHA-256 digest. */
  [[nodiscard]] static PresetIdentity from_digest(
    std::string preset_id, std::uint32_t schema_version,
    std::string canonical_digest);

  [[nodiscard]] const std::string & preset_id() const noexcept;
  [[nodiscard]] std::uint32_t schema_version() const noexcept;
  [[nodiscard]] const std::string & canonical_digest() const noexcept;

  [[nodiscard]] bool operator==(const PresetIdentity & other) const noexcept;
  [[nodiscard]] bool operator!=(const PresetIdentity & other) const noexcept;

private:
  PresetIdentity(
    std::string preset_id, std::uint32_t schema_version,
    std::string canonical_digest);

  std::string preset_id_;
  std::uint32_t schema_version_;
  std::string canonical_digest_;
};

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__PRESET_IDENTITY_HPP_
