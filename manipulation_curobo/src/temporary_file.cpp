// Responsibility: Creates restrictive temporary files and removes only files owned by the instance.

#include "manipulation_curobo/temporary_file.hpp"

#include <unistd.h>

#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace manipulation_curobo::curobo
{
namespace
{

bool validPrefix(const std::string_view prefix)
{
  if (prefix.empty()) {
    return false;
  }
  for (const unsigned char character : prefix) {
    if (std::isalnum(character) == 0 && character != '_' && character != '-') {
      return false;
    }
  }
  return true;
}

}  // namespace

TemporaryFile::TemporaryFile(const std::string_view prefix)
{
  if (!validPrefix(prefix)) {
    throw std::invalid_argument("temporary-file prefix is not a safe identifier");
  }

  const auto template_path =
    std::filesystem::temp_directory_path() / (std::string{prefix} + "_XXXXXX");
  std::string template_string = template_path.string();
  std::vector<char> writable_template(template_string.begin(), template_string.end());
  writable_template.push_back('\0');

  const int descriptor = ::mkstemp(writable_template.data());
  if (descriptor < 0) {
    throw std::system_error(
            errno, std::generic_category(), "failed to create private temporary file");
  }

  path_ = std::filesystem::path{writable_template.data()};
  if (::close(descriptor) != 0) {
    const int close_error = errno;
    removeOwnedFile();
    throw std::system_error(
            close_error, std::generic_category(), "failed to close temporary file");
  }
}

TemporaryFile::~TemporaryFile() noexcept
{
  removeOwnedFile();
}

TemporaryFile::TemporaryFile(TemporaryFile && other) noexcept
: path_(std::exchange(other.path_, std::filesystem::path {}))
{
}

TemporaryFile & TemporaryFile::operator=(TemporaryFile && other) noexcept
{
  if (this != &other) {
    removeOwnedFile();
    path_ = std::exchange(other.path_, std::filesystem::path{});
  }
  return *this;
}

const std::filesystem::path & TemporaryFile::path() const noexcept
{
  return path_;
}

void TemporaryFile::removeOwnedFile() noexcept
{
  if (path_.empty()) {
    return;
  }
  std::error_code ignored;
  std::filesystem::remove(path_, ignored);
  path_.clear();
}

}  // namespace manipulation_curobo::curobo
