#include "mw/message.hpp"

#include "mw/time.hpp"

namespace mw {

std::uint64_t stamp(proto::Header& header,
                    std::string_view publisher,
                    std::uint64_t& sequence) noexcept {
  const std::uint64_t n = sequence++;

  header.set_schema_version(kSchemaVersion);
  header.set_publisher(std::string{publisher});
  header.set_sequence(n);
  header.set_publish_time_ns(wall_time_ns());

  return n;
}

HeaderStatus peek_header(std::string_view bytes, proto::Header& out) noexcept {
  proto::AnyHeader any;

  // ParseFromArray, not ParseFromString: string_view is not null-terminated and
  // we already know the length. The cast is needed because protobuf takes an
  // int size, and a string_view's size() is a size_t.
  if (!any.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
    return HeaderStatus::kNotParsable;
  }

  // In proto3 a submessage is genuinely optional: has_header() distinguishes
  // "absent" from "present but every field is zero".
  if (!any.has_header()) {
    return HeaderStatus::kMissingHeader;
  }

  const std::uint32_t v = any.header().schema_version();
  if (v > kSchemaVersion) return HeaderStatus::kSchemaTooNew;
  if (v < kSchemaVersion) return HeaderStatus::kSchemaTooOld;

  out = any.header();
  return HeaderStatus::kOk;
}

std::string_view to_string(HeaderStatus status) noexcept {
  switch (status) {
    case HeaderStatus::kOk:            return "ok";
    case HeaderStatus::kNotParsable:   return "not parsable";
    case HeaderStatus::kMissingHeader: return "missing header";
    case HeaderStatus::kSchemaTooNew:  return "schema too new";
    case HeaderStatus::kSchemaTooOld:  return "schema too old";
  }
  return "unknown";
}

}  // namespace mw
