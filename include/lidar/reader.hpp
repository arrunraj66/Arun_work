#pragma once
//
// lidar/reader.hpp  ScanReader, Step 7: read back what ScanRecorder wrote.
// This is the other half of Concept 3's log+index split: the recorder only
// ever appends, the reader only ever looks things up. Same pimpl shape as
// ScanRecorder, for the same reason -- neither SQLite nor protobuf may leak
// into a header the GUI team includes.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "lidar/scan.hpp"

namespace lidar {

/// Reads scans back out of the log+index pair a ScanRecorder produced.
/// Opens both files read-only; never writes to either.
class ScanReader {
 public:
  ScanReader(std::string log_path, std::string index_db_path);
  ~ScanReader();

  ScanReader(const ScanReader&) = delete;
  ScanReader& operator=(const ScanReader&) = delete;
  ScanReader(ScanReader&&) noexcept;
  ScanReader& operator=(ScanReader&&) noexcept;

  /// How many scans are indexed, total.
  [[nodiscard]] std::uint64_t count() const;

  /// The scan at position `index`, counting from 0 in recording order (the
  /// same order ScanRecorder wrote them in). Throws std::out_of_range if
  /// `index >= count()`.
  [[nodiscard]] Scan get(std::uint64_t index) const;

  /// Every scan whose stamp_ns falls in [start_stamp_ns, end_stamp_ns],
  /// inclusive on both ends, in recording order. Empty if none match.
  [[nodiscard]] std::vector<Scan> find_by_time_range(std::int64_t start_stamp_ns,
                                                       std::int64_t end_stamp_ns) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar