// Responsibility: Defines the canonical, side-effect-free syntax policy for ROS frame IDs.

#ifndef MANIPULATION__VALIDATION__FRAME_ID_HPP_
#define MANIPULATION__VALIDATION__FRAME_ID_HPP_

#include <cctype>
#include <string>
#include <string_view>

namespace manipulation::validation
{

/**
 * Return true when a frame ID has one canonical relative spelling.
 *
 * Nested frames may use `/` separators. Leading, trailing, or repeated
 * separators are rejected together with whitespace and control characters.
 * REP-105 defines frame semantics rather than this syntax; this project policy
 * prevents aliases such as `/world`, `world/`, and `robot//camera` at API
 * boundaries.
 */
[[nodiscard]] inline bool is_canonical_relative_frame_id(
  const std::string_view frame_id) noexcept
{
  if (frame_id.empty() || frame_id.front() == '/' || frame_id.back() == '/') {
    return false;
  }

  bool previous_was_separator = false;
  for (const char raw_character : frame_id) {
    const auto character = static_cast<unsigned char>(raw_character);
    const bool separator = character == '/';
    if (std::isspace(character) != 0 || std::iscntrl(character) != 0 ||
      (separator && previous_was_separator))
    {
      return false;
    }
    previous_was_separator = separator;
  }
  return true;
}

/** Preserve the original public name while consumers migrate to the precise policy name. */
[[nodiscard]] inline bool is_relative_rep105_frame_id(
  const std::string & frame_id)
{
  return is_canonical_relative_frame_id(frame_id);
}

}  // namespace manipulation::validation

#endif  // MANIPULATION__VALIDATION__FRAME_ID_HPP_
