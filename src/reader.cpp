#include "lidar/reader.hpp"

#include <sqlite3.h>

#include <fstream>
#include <stdexcept>
#include <string>

#include "scan_codec.hpp"

namespace lidar {

namespace {

// The inverse of write_length_prefix() in recorder.cpp: four explicit
// little-endian bytes back into a uint32_t. Never reinterpret_cast the raw
// bytes -- same reasoning as the write side, just run backwards.
std::uint32_t read_length_prefix(const unsigned char bytes[4]) {
  return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8) |
         (static_cast<std::uint32_t>(bytes[2]) << 16) | (static_cast<std::uint32_t>(bytes[3]) << 24);
}

void check_sqlite(int rc, sqlite3* db, const char* what) {
  if (rc != SQLITE_OK && rc != SQLITE_DONE && rc != SQLITE_ROW) {
    const std::string message =
        std::string("ScanReader: ") + what + " failed: " + sqlite3_errmsg(db);
    throw std::runtime_error(message);
  }
}

}  // namespace

struct ScanReader::Impl {
  Impl(const std::string& log_path, const std::string& index_db_path) {
    try {
    log.open(log_path, std::ios::binary);
    if (!log.is_open()) {
      throw std::runtime_error("ScanReader: could not open log file " + log_path);
    }

    check_sqlite(sqlite3_open_v2(index_db_path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr), db,
                 "sqlite3_open_v2");

    static constexpr char kCount[] = "SELECT COUNT(*) FROM scans;";
    check_sqlite(sqlite3_prepare_v2(db, kCount, -1, &count_stmt, nullptr), db,
                 "sqlite3_prepare_v2(COUNT)");

    // seq is the table's INTEGER PRIMARY KEY -- SQLite makes that an alias
    // for the row's own rowid, so it is exactly 1, 2, 3, ... in insertion
    // order with no gaps, which is what lets get(index) below turn a plain
    // 0-based position into a `WHERE seq = ?` lookup.
    static constexpr char kGetBySeq[] =
        "SELECT stamp_ns, offset, length FROM scans WHERE seq = ?;";
    check_sqlite(sqlite3_prepare_v2(db, kGetBySeq, -1, &get_stmt, nullptr), db,
                 "sqlite3_prepare_v2(GET)");

    static constexpr char kRange[] =
        "SELECT stamp_ns, offset, length FROM scans"
        " WHERE stamp_ns >= ? AND stamp_ns <= ? ORDER BY seq;";
    check_sqlite(sqlite3_prepare_v2(db, kRange, -1, &range_stmt, nullptr), db,
                 "sqlite3_prepare_v2(RANGE)");
    } catch (...) {
      close_resources();
      throw;
    }
  }

  void close_resources() noexcept {
    if (range_stmt != nullptr) sqlite3_finalize(range_stmt);
    range_stmt = nullptr;
    if (get_stmt != nullptr) sqlite3_finalize(get_stmt);
    get_stmt = nullptr;
    if (count_stmt != nullptr) sqlite3_finalize(count_stmt);
    count_stmt = nullptr;
    if (db != nullptr) sqlite3_close(db);
    db = nullptr;
  }

  ~Impl() {
    close_resources();
    // log closes itself: std::ifstream's destructor closes it.
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  // Reads the record whose index row said "offset, length" and turns it
  // back into a Scan. Shared by get() and find_by_time_range() so there is
  // exactly one place that knows the on-disk record shape.
  Scan read_record(std::int64_t offset, std::int64_t length) {
    log.seekg(offset, std::ios::beg);
    if (!log.good()) {
      throw std::runtime_error("ScanReader: seek to offset " + std::to_string(offset) + " failed");
    }

    unsigned char prefix[4];
    log.read(reinterpret_cast<char*>(prefix), sizeof(prefix));
    if (log.gcount() != static_cast<std::streamsize>(sizeof(prefix))) {
      // Concept 3's documented case: a final record cut short by a crash
      // mid-write. It should never happen for a row that made it into the
      // index (log write happens, and only then the index insert), but a
      // reader has to treat it as "no more usable data," not corruption.
      throw std::runtime_error("ScanReader: log is shorter than the index says -- truncated record");
    }

    const std::uint32_t declared_length = read_length_prefix(prefix);
    if (declared_length != static_cast<std::uint32_t>(length)) {
      throw std::runtime_error("ScanReader: log's length prefix (" + std::to_string(declared_length) +
                                ") does not match the index's length column (" +
                                std::to_string(length) + ") at offset " + std::to_string(offset));
    }

    std::string payload(declared_length, '\0');
    log.read(payload.data(), static_cast<std::streamsize>(declared_length));
    if (static_cast<std::uint32_t>(log.gcount()) != declared_length) {
      throw std::runtime_error("ScanReader: log is shorter than the index says -- truncated payload");
    }

    if (!wire.ParseFromString(payload)) {
      throw std::runtime_error("ScanReader: payload at offset " + std::to_string(offset) +
                                " does not parse as a LidarScan");
    }
    return from_proto(wire);
  }

  std::ifstream log;
  sqlite3* db = nullptr;
  sqlite3_stmt* count_stmt = nullptr;
  sqlite3_stmt* get_stmt = nullptr;
  sqlite3_stmt* range_stmt = nullptr;

  // Reused across calls -- same reasoning as ScanRecorder::Impl::wire: one
  // allocation, not one per scan read back.
  proto::LidarScan wire;
};

ScanReader::ScanReader(std::string log_path, std::string index_db_path)
    : impl_(std::make_unique<Impl>(log_path, index_db_path)) {}

ScanReader::~ScanReader() = default;
ScanReader::ScanReader(ScanReader&&) noexcept = default;
ScanReader& ScanReader::operator=(ScanReader&&) noexcept = default;

std::uint64_t ScanReader::count() const {
  sqlite3_reset(impl_->count_stmt);
  check_sqlite(sqlite3_step(impl_->count_stmt), impl_->db, "sqlite3_step(COUNT)");
  return static_cast<std::uint64_t>(sqlite3_column_int64(impl_->count_stmt, 0));
}

Scan ScanReader::get(std::uint64_t index) const {
  if (index >= count()) {
    throw std::out_of_range("ScanReader::get: index " + std::to_string(index) + " >= count()");
  }

  // seq is 1-based and contiguous (see the comment in Impl's constructor),
  // so the 0-based `index` a caller thinks in maps to seq = index + 1.
  const auto seq = static_cast<std::int64_t>(index) + 1;

  sqlite3_reset(impl_->get_stmt);
  sqlite3_bind_int64(impl_->get_stmt, 1, seq);
  check_sqlite(sqlite3_step(impl_->get_stmt), impl_->db, "sqlite3_step(GET)");

  const std::int64_t offset = sqlite3_column_int64(impl_->get_stmt, 1);
  const std::int64_t length = sqlite3_column_int64(impl_->get_stmt, 2);
  return impl_->read_record(offset, length);
}

std::vector<Scan> ScanReader::find_by_time_range(std::int64_t start_stamp_ns,
                                                   std::int64_t end_stamp_ns) const {
  std::vector<Scan> result;

  sqlite3_reset(impl_->range_stmt);
  sqlite3_bind_int64(impl_->range_stmt, 1, start_stamp_ns);
  sqlite3_bind_int64(impl_->range_stmt, 2, end_stamp_ns);

  while (true) {
    const int rc = sqlite3_step(impl_->range_stmt);
    if (rc == SQLITE_DONE) break;
    check_sqlite(rc, impl_->db, "sqlite3_step(RANGE)");

    const std::int64_t offset = sqlite3_column_int64(impl_->range_stmt, 1);
    const std::int64_t length = sqlite3_column_int64(impl_->range_stmt, 2);
    result.push_back(impl_->read_record(offset, length));
  }

  return result;
}

}  // namespace lidar