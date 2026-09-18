#pragma once
//
// lidar/recorder.hpp  ScanRecorder, Concept 3 from Lesson 2 turned into
// code: an append-only log holds the scans, a SQLite index holds one row
// per scan (sequence, timestamp, byte offset, byte length, sample count)
// so a reader can later find "the scan near time T" without reading the
// whole log from the start.
//
// Neither SQLite nor protobuf appears in this header -- both are pimpl'd
// away in src/recorder.cpp, same PUBLIC/PRIVATE discipline as everything
// else the GUI team links against.

#include <cstdint>
#include <memory>
#include <string>

#include "lidar/scan.hpp"

namespace lidar {

/// Appends Scans to a length-prefixed binary log and indexes each one in
/// SQLite. Both files are opened for append, not overwrite -- recording
/// again later adds on to what's already there.
class ScanRecorder {
 public:
  ScanRecorder(std::string log_path, std::string index_db_path);
  ~ScanRecorder();

  ScanRecorder(const ScanRecorder&) = delete;
  ScanRecorder& operator=(const ScanRecorder&) = delete;
  ScanRecorder(ScanRecorder&&) noexcept;
  ScanRecorder& operator=(ScanRecorder&&) noexcept;

  /// Serialises `scan`, appends it to the log, and indexes it -- one call
  /// does both halves of Concept 3's split, so a caller can never write to
  /// the log and forget to index it, or the reverse.
  void record(const Scan& scan);

  /// Total scans indexed so far: whatever the log already held when this
  /// recorder was opened, plus everything recorded this run.
  [[nodiscard]] std::uint64_t count() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar