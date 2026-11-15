// Responsibility: Generates operation IDs and stable cryptographic digests for idempotent scene
//   writes.

#include "manipulation/scene/coordinator_utils.hpp"

#include <openssl/evp.h>
#include <openssl/rand.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace manipulation::scene
{
namespace
{

std::string bytes_to_hex(const unsigned char * bytes, const std::size_t size)
{
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return output.str();
}

std::string sha256_hex(const unsigned char * data, const std::size_t size)
{
  using Context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), data, size) != 1)
  {
    throw std::runtime_error("failed to initialize or update OpenSSL SHA-256");
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("failed to finalize OpenSSL SHA-256");
  }
  return bytes_to_hex(digest.data(), digest_size);
}

std::string text_digest(const std::string & canonical_text)
{
  return sha256_hex(
    reinterpret_cast<const unsigned char *>(canonical_text.data()),
    canonical_text.size());
}

}  // namespace

std::string digest_apply_request(
  const manipulation_msgs::srv::ApplyPlanningSceneDiff::Request & request)
{
  auto canonical_request = request;
  std::sort(
    canonical_request.resource_keys.begin(), canonical_request.resource_keys.end());

  // Generated ROSIDL YAML walks fields in schema order and does not contain
  // transport alignment padding. Raw CDR padding is unspecified and therefore
  // cannot be part of a deterministic idempotency key.
  return text_digest(manipulation_msgs::srv::to_yaml(canonical_request, true));
}

std::string digest_authoritative_scene(const moveit_msgs::msg::PlanningScene & scene)
{
  return text_digest(
    moveit_msgs::msg::to_yaml(canonical_authoritative_scene(scene), true));
}

std::string generate_session_id()
{
  std::array<unsigned char, 16> bytes{};
  if (RAND_bytes(bytes.data(), bytes.size()) != 1) {
    throw std::runtime_error("OpenSSL failed to generate a coordinator session UUID");
  }

  // RFC 4122 version 4 and variant 1 bits make the session recognizable as a UUID.
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3fU) | 0x80U);

  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index == 4U || index == 6U || index == 8U || index == 10U) {
      output << '-';
    }
    output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return output.str();
}

}  // namespace manipulation::scene
