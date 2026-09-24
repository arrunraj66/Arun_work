#include "lidar/query_server.hpp"

#include <charconv>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lidar/reader.hpp"
#include "mw/transport.hpp"
#include "query_wire.hpp"

namespace lidar {
namespace {

// Parses "RANGE <start> <end>" into the two integers. Returns false if the
// command is not shaped that way -- covers both a genuinely different
// command and a client sending garbage, and both get the same answer: the
// caller's request cannot be understood, so nothing is queried for it.
bool parse_range(const std::string& command, std::int64_t& start_ns, std::int64_t& end_ns) {
  if (command.rfind("RANGE ", 0) != 0) return false;

  const char* begin = command.data() + 6;
  const char* end   = command.data() + command.size();

  const auto r1 = std::from_chars(begin, end, start_ns);
  if (r1.ec != std::errc()) return false;

  const char* mid = r1.ptr;
  while (mid != end && *mid == ' ') ++mid;   // skip the space between numbers

  const auto r2 = std::from_chars(mid, end, end_ns);
  return r2.ec == std::errc();
}

// Parses "RECENT <window_ns>" into the single integer. Same shape and same
// "false means not this command" contract as parse_range() above.
bool parse_recent(const std::string& command, std::int64_t& window_ns) {
  if (command.rfind("RECENT ", 0) != 0) return false;

  const char* begin = command.data() + 7;
  const char* end   = command.data() + command.size();

  const auto r = std::from_chars(begin, end, window_ns);
  return r.ec == std::errc();
}

}  // namespace

struct ScanQueryServer::Impl {
  ScanReader   reader;     // read-only view of the same log+index the
                           // recorder is writing -- see reader.hpp
  mw::Context  ctx;
  mw::Replier  replier;
  std::uint64_t served_count = 0;

  Impl(std::string log_path, std::string index_db_path, std::string endpoint)
      : reader(std::move(log_path), std::move(index_db_path)), replier(ctx, endpoint) {}

  // The one place both branches of the protocol live. Returns the framed
  // reply bytes -- never throws past this point, because a malformed
  // request must still get *some* reply, or the client-side REQ socket
  // would be left waiting past its own timeout for nothing.
  std::string handle(const std::string& command) {
    ++served_count;

    if (command == "LATEST") {
      std::vector<Scan> out;
      if (reader.count() > 0) out.push_back(reader.get(reader.count() - 1));
      return encode_scans(out);
    }

    std::int64_t start_ns = 0, end_ns = 0;
    if (parse_range(command, start_ns, end_ns)) {
      return encode_scans(reader.find_by_time_range(start_ns, end_ns));
    }

    // "The last window_ns of whatever's been recorded" -- anchored to the
    // NEWEST RECORDED scan's own stamp_ns, not this machine's wall clock.
    // Recording keeps everything forever (no deletion happens here); this
    // is purely a read-side view over the tail of it. An empty recording
    // has no "newest" to anchor to, so it answers "nothing" the same way
    // LATEST does in that case, rather than treating window_ns as an
    // absolute time and returning a meaningless empty range.
    std::int64_t window_ns = 0;
    if (parse_recent(command, window_ns)) {
      if (reader.count() == 0) return encode_scans({});
      const std::int64_t newest_stamp_ns = reader.get(reader.count() - 1).stamp_ns;
      return encode_scans(reader.find_by_time_range(newest_stamp_ns - window_ns, newest_stamp_ns));
    }

    // Unrecognised command: answered with an honest "nothing", not silence.
    // A REP socket that never replies at all would leave the client's
    // Requester stuck until its own timeout does the same job less cleanly.
    return encode_scans({});
  }
};

ScanQueryServer::ScanQueryServer(std::string log_path, std::string index_db_path,
                                 std::string endpoint)
    : impl_(std::make_unique<Impl>(std::move(log_path), std::move(index_db_path),
                                   std::move(endpoint))) {}

ScanQueryServer::~ScanQueryServer()                                     = default;
ScanQueryServer::ScanQueryServer(ScanQueryServer&&) noexcept            = default;
ScanQueryServer& ScanQueryServer::operator=(ScanQueryServer&&) noexcept = default;

bool ScanQueryServer::serve_one(int timeout_ms) {
  return impl_->replier.serve_one(timeout_ms,
                                  [this](const std::string& req) { return impl_->handle(req); });
}

std::uint64_t ScanQueryServer::served() const noexcept { return impl_->served_count; }

}  // namespace lidar
