// Responsibility: Provides exclusive temporary-file ownership for planner request and response
//   exchange.

#ifndef MANIPULATION_CUROBO__TEMPORARY_FILE_HPP_
#define MANIPULATION_CUROBO__TEMPORARY_FILE_HPP_

#include <filesystem>
#include <string_view>

namespace manipulation_curobo::curobo
{

/** Own a private temporary file and unlink it on every scope-exit path. */
class TemporaryFile final
{
public:
  /**
   * Create an empty mode-0600 file in the system temporary directory.
   *
   * The prefix accepts only ASCII letters, digits, `_` and `-`; path
   * separators are rejected. Construction throws on creation or close errors.
   */
  explicit TemporaryFile(std::string_view prefix);
  ~TemporaryFile() noexcept;

  TemporaryFile(const TemporaryFile &) = delete;
  TemporaryFile & operator=(const TemporaryFile &) = delete;
  TemporaryFile(TemporaryFile && other) noexcept;
  TemporaryFile & operator=(TemporaryFile && other) noexcept;

  /// Return the owned path. The file remains valid until this object moves or dies.
  [[nodiscard]] const std::filesystem::path & path() const noexcept;

private:
  void removeOwnedFile() noexcept;

  std::filesystem::path path_;
};

}  // namespace manipulation_curobo::curobo

#endif  // MANIPULATION_CUROBO__TEMPORARY_FILE_HPP_
