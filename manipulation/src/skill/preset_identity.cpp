// Responsibility: Validates preset identities and calculates deterministic SHA-256 digests.

#include "manipulation/skill/preset_identity.hpp"

#include <array>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "manipulation/skill/logical_id.hpp"
#include "openssl/evp.h"

namespace manipulation::skill
{
namespace
{

std::string sha256(const std::string_view content)
{
  using Context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), content.data(), content.size()) != 1)
  {
    throw std::runtime_error("Failed to initialize the preset SHA-256 digest");
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("Failed to finalize the preset SHA-256 digest");
  }

  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0');
  for (std::size_t index = 0U; index < digest_size; ++index) {
    encoded << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return encoded.str();
}

bool is_lowercase_sha256(const std::string_view digest)
{
  if (digest.size() != 64U) {
    return false;
  }
  for (const char character : digest) {
    const bool is_digit = character >= '0' && character <= '9';
    const bool is_lower_hex = character >= 'a' && character <= 'f';
    if (!is_digit && !is_lower_hex) {
      return false;
    }
  }
  return true;
}

}  // namespace

PresetIdentity PresetIdentity::from_canonical_content(
  std::string preset_id, const std::uint32_t schema_version,
  const std::string_view canonical_content)
{
  return PresetIdentity(
    std::move(preset_id), schema_version, sha256(canonical_content));
}

PresetIdentity PresetIdentity::from_digest(
  std::string preset_id, const std::uint32_t schema_version,
  std::string canonical_digest)
{
  return PresetIdentity(
    std::move(preset_id), schema_version, std::move(canonical_digest));
}

const std::string & PresetIdentity::preset_id() const noexcept
{
  return preset_id_;
}

std::uint32_t PresetIdentity::schema_version() const noexcept
{
  return schema_version_;
}

const std::string & PresetIdentity::canonical_digest() const noexcept
{
  return canonical_digest_;
}

bool PresetIdentity::operator==(const PresetIdentity & other) const noexcept
{
  return preset_id_ == other.preset_id_ && schema_version_ == other.schema_version_ &&
         canonical_digest_ == other.canonical_digest_;
}

bool PresetIdentity::operator!=(const PresetIdentity & other) const noexcept
{
  return !(*this == other);
}

PresetIdentity::PresetIdentity(
  std::string preset_id, const std::uint32_t schema_version,
  std::string canonical_digest)
: preset_id_(std::move(preset_id)),
  schema_version_(schema_version),
  canonical_digest_(std::move(canonical_digest))
{
  if (!is_canonical_logical_id(preset_id_)) {
    throw std::invalid_argument("Preset ID must be a canonical logical ID");
  }
  if (schema_version_ == 0U) {
    throw std::invalid_argument("Preset schema version must be greater than zero");
  }
  if (!is_lowercase_sha256(canonical_digest_)) {
    throw std::invalid_argument("Preset digest must be a lowercase SHA-256 value");
  }
}

}  // namespace manipulation::skill
