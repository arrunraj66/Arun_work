// Public header. Anything in include/mw/ is API: other people compile against
// it, so every change here is a change to somebody else's build.
//
// Rule for this directory: no ZeroMQ, no Protobuf, no SQLite. Ever.

#pragma once

#include <string_view>

namespace mw {

/// The version of the middleware this binary was built from.
struct Version {
  int major;
  int minor;
  int patch;
};

/// The version as three numbers.
[[nodiscard]] Version version() noexcept;

/// The version as text, e.g. "0.1.0". Points at a string literal with static
/// storage duration, so the view never dangles.
[[nodiscard]] std::string_view version_string() noexcept;

}  // namespace mw
