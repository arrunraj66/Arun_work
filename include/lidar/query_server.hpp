#pragma once
//
// lidar/query_server.hpp  ScanQueryServer, the other end of ScanQueryClient.
//
// Runs on the machine that owns the recording (typically the same machine
// as scan_publisher_main, but it only needs the log+index -- it never
// touches the sensor). Answers exactly two questions, read-only, straight
// out of the same log+index ScanRecorder writes and ScanReader reads:
// "what is the latest scan" and "what scans fall in this time range".
//
// One call, one request, same shape as Executor::step() and
// IScanSource::poll_scan(): do one unit of work, report whether there was
// any to do, never block longer than asked, never throw across the call.

#include <cstdint>
#include <memory>
#include <string>

namespace lidar {

class ScanQueryServer {
 public:
  /// Opens `log_path`/`index_db_path` read-only (same files ScanRecorder is
  /// writing, or was writing) and binds `endpoint` to answer requests on.
  /// Throws if either file cannot be opened or `endpoint` cannot be bound --
  /// the same "fail before anything is running" reasoning as every other
  /// constructor in this library.
  ScanQueryServer(std::string log_path, std::string index_db_path, std::string endpoint);
  ~ScanQueryServer();

  ScanQueryServer(const ScanQueryServer&)            = delete;
  ScanQueryServer& operator=(const ScanQueryServer&) = delete;
  ScanQueryServer(ScanQueryServer&&) noexcept;
  ScanQueryServer& operator=(ScanQueryServer&&) noexcept;

  /// Wait up to `timeout_ms` for one request. If one arrives, answer it and
  /// return true. False on timeout -- the ordinary idle case, not an error.
  /// A caller runs this in a loop, same shape as any poll-based read.
  [[nodiscard]] bool serve_one(int timeout_ms);

  /// How many requests have been answered since construction -- including
  /// ones that matched nothing, which still count as answered.
  [[nodiscard]] std::uint64_t served() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar