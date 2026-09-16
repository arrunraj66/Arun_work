#include "mw/version.hpp"

// MW_VERSION_* are injected by CMake from the project() version, so the number
// lives in exactly one place: the top-level CMakeLists.txt.

namespace mw {

Version version() noexcept {
  return Version{MW_VERSION_MAJOR, MW_VERSION_MINOR, MW_VERSION_PATCH};
}

std::string_view version_string() noexcept {
  return MW_VERSION_STRING;
}

}  // namespace mw
