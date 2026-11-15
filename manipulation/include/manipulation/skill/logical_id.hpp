// Responsibility: Validates canonical slash-separated identifiers shared by skill metadata.

#ifndef MANIPULATION__SKILL__LOGICAL_ID_HPP_
#define MANIPULATION__SKILL__LOGICAL_ID_HPP_

#include <string_view>

namespace manipulation::skill
{

/**
 * Return true for a canonical lowercase logical identifier.
 *
 * An identifier contains one or more slash-separated segments. Every segment
 * starts with an ASCII lowercase letter and continues with lowercase letters,
 * digits, underscores, dots, or hyphens. Leading, trailing, and repeated
 * slashes are therefore rejected. The function is locale-independent.
 */
[[nodiscard]] inline bool is_canonical_logical_id(const std::string_view value) noexcept
{
  if (value.empty()) {
    return false;
  }

  bool segment_start = true;
  for (const char character : value) {
    if (character == '/') {
      if (segment_start) {
        return false;
      }
      segment_start = true;
      continue;
    }

    const bool is_lowercase = character >= 'a' && character <= 'z';
    if (segment_start) {
      if (!is_lowercase) {
        return false;
      }
      segment_start = false;
      continue;
    }

    const bool is_digit = character >= '0' && character <= '9';
    const bool is_separator =
      character == '_' || character == '.' || character == '-';
    if (!is_lowercase && !is_digit && !is_separator) {
      return false;
    }
  }
  return !segment_start;
}

}  // namespace manipulation::skill

#endif  // MANIPULATION__SKILL__LOGICAL_ID_HPP_
