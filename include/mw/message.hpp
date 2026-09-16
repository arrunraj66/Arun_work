// Everything to do with the message header, in one place.
//
// Note what this header includes: mw/envelope.pb.h, a generated protobuf type.
// That is a deliberate exception to the "no third-party headers in public
// headers" rule, and the reasoning is written down in docs/adr/0001.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "mw/envelope.pb.h"

namespace mw {

/// The schema version this build was compiled against.
///
/// Bump it whenever the meaning of an existing field changes. Adding a new
/// optional field does not need a bump — that is the whole point of protobuf.
inline constexpr std::uint32_t kSchemaVersion = 1;

/// Fill in a header ready to publish. Returns the sequence number used.
///
/// `sequence` is taken by reference and incremented, so the caller keeps one
/// counter per topic and this function owns the increment.
std::uint64_t stamp(proto::Header& header,
                    std::string_view publisher,
                    std::uint64_t& sequence) noexcept;

/// Why a message was rejected. Anything other than kOk means do not use it.
enum class HeaderStatus {
  kOk,
  kNotParsable,       ///< the bytes are not a protobuf message at all
  kMissingHeader,     ///< parsed, but field 1 was absent
  kSchemaTooNew,      ///< published by a newer build than this one
  kSchemaTooOld,      ///< published by a build older than we still accept
};

/// Read the header out of a serialised message of ANY type.
///
/// Works because every message in this system puts its Header in field 1, and
/// protobuf ignores fields it does not know about. The logger uses this to
/// timestamp topics whose payload type it has never been compiled against.
[[nodiscard]] HeaderStatus peek_header(std::string_view bytes,
                                       proto::Header& out) noexcept;

/// Human-readable form of a status, for logs and test failures.
[[nodiscard]] std::string_view to_string(HeaderStatus status) noexcept;

}  // namespace mw
