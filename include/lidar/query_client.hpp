#pragma once
//
// lidar/query_client.hpp  ScanQueryClient, the backup path to ScanSubscriber.
//
// ScanSubscriber (lidar/subscriber.hpp) is push-based: it sees only what was
// published WHILE it was listening, and a scan published before it connected
// or during a network drop is simply gone. That is fine for a live view, but
// it means there is no way to ask "what did I miss" -- there is nothing to
// ask, because a PUB socket does not remember.
//
// ScanQueryClient is pull-based instead: it asks a ScanQueryServer for
// exactly what it wants, whenever it wants it, and the server answers from
// its recording. Use it when the live stream is unavailable, or to backfill
// whatever a subscriber missed while it was down.
//
// It is NOT a replacement for ScanSubscriber in ordinary operation -- every
// call here blocks up to a timeout waiting for a reply, which is a poor fit
// for a 25 Hz live feed. Pull for gaps and history; subscribe for the feed.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lidar/scan.hpp"

namespace lidar {

class ScanQueryClient {
 public:
  /// `endpoint` is the ScanQueryServer's address, e.g.
  /// "tcp://192.168.12.240:5560". `timeout_ms` applies to every call below;
  /// a call that gets no reply within it returns "nothing", exactly like a
  /// poll_scan() timeout, and ok() reports which kind of nothing it was.
  explicit ScanQueryClient(std::string endpoint, int timeout_ms = 2000);
  ~ScanQueryClient();

  ScanQueryClient(const ScanQueryClient&)            = delete;
  ScanQueryClient& operator=(const ScanQueryClient&) = delete;
  ScanQueryClient(ScanQueryClient&&) noexcept;
  ScanQueryClient& operator=(ScanQueryClient&&) noexcept;

  /// The most recently recorded scan, or nothing if the recording is empty
  /// OR the server could not be reached -- call ok() right after to tell
  /// those two apart.
  [[nodiscard]] std::optional<Scan> latest() noexcept;

  /// Every scan whose stamp_ns falls in [start_ns, end_ns], inclusive, in
  /// recording order -- same contract as ScanReader::find_by_time_range(),
  /// just answered by a server instead of read from a local file. Empty
  /// means either nothing matched OR the server could not be reached; call
  /// ok() to tell those apart.
  [[nodiscard]] std::vector<Scan> range(std::int64_t start_ns, std::int64_t end_ns) noexcept;
  /// A rolling window: every scan recorded within `window_ns` of the MOST
  /// RECENTLY RECORDED scan (not wall-clock "now" -- deliberately, so this
  /// still means something sensible hours after recording stopped, or if
  /// this machine's clock and the sensor's data disagree). Recording keeps
  /// everything forever; this is a VIEW over the last `window_ns` of it,
  /// nothing is deleted by asking for it. Equivalent to calling range() with
  /// [newest_stamp_ns - window_ns, newest_stamp_ns] yourself, except the
  /// server works out `newest_stamp_ns` for you so you don't need a
  /// separate latest() round trip first. Empty means either nothing in that
  /// window OR the server could not be reached; call ok() to tell those
  /// apart.
  [[nodiscard]] std::vector<Scan> recent(std::int64_t window_ns) noexcept;
  /// Whether the MOST RECENT call above actually reached the server and got
  /// a well-formed reply. False after latest()/range() means what came back
  /// (nullopt / empty) says nothing about whether data existed -- the
  /// request simply never got an answer.
  [[nodiscard]] bool ok() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar