#include "lidar/recorder.hpp"

#include <sqlite3.h>

#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

#include "scan_codec.hpp"

namespace lidar {

namespace {

// Writes a 32-bit length as four explicit bytes, little-endian, regardless
// of the host's own byte order -- Concept 3's rule: "the file must mean the
// same thing everywhere," not just on whatever machine wrote it.
void write_length_prefix(std::ofstream& out, std::uint32_t length) {
  char bytes[4];
  bytes[0] = static_cast<char>(length & 0xFFU);
  bytes[1] = static_cast<char>((length >> 8) & 0xFFU);
  bytes[2] = static_cast<char>((length >> 16) & 0xFFU);
  bytes[3] = static_cast<char>((length >> 24) & 0xFFU);
  out.write(bytes, sizeof(bytes));
}

void check_sqlite(int rc, sqlite3* db, const char* what) {
  if (rc != SQLITE_OK && rc != SQLITE_DONE && rc != SQLITE_ROW) {
    const std::string message =
        std::string("ScanRecorder: ") + what + " failed: " + sqlite3_errmsg(db);
    throw std::runtime_error(message);
  }
}

}  // namespace

struct ScanRecorder::Impl {
  Impl(const std::string& log_path, const std::string& index_db_path) {
    try {
    // ios::app, not ios::out: every write lands at the current end of file
    // regardless of any earlier seek, which is what lets recording resume
    // across separate runs without corrupting what's already logged.
    log.open(log_path, std::ios::binary | std::ios::app);
    if (!log.is_open()) {
      throw std::runtime_error("ScanRecorder: could not open log file " + log_path);
    }

    check_sqlite(sqlite3_open(index_db_path.c_str(), &db), db, "sqlite3_open");

    // Two fixes for the same underlying problem, both needed together.
    //
    // SQLite's DEFAULT journal mode requires a writer to hold an EXCLUSIVE
    // lock on the whole database file for the duration of a write
    // transaction, during which no reader can even start one. The moment a
    // second concurrent reader existed against this file (once
    // lidar_monitor_main's own ScanReader joined scan_query_server_main's),
    // an INSERT landing at the same instant as a SELECT was no longer a
    // rare coincidence -- it happened within minutes, and with no retry
    // configured, SQLite fails it immediately rather than waiting even a
    // few milliseconds: "database is locked".
    //
    // WAL (write-ahead log) mode is SQLite's own fix for exactly this: one
    // writer and any number of readers proceed concurrently without
    // blocking each other, because readers see a consistent snapshot from
    // the WAL file instead of needing the writer's lock at all. It is a
    // property of the database FILE, not the connection -- setting it here,
    // once, upgrades an existing file too (readers opening it later, even
    // read-only, automatically see WAL mode from the file header).
    check_sqlite(sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, nullptr), db,
                 "PRAGMA journal_mode=WAL");

    // Defense in depth on top of WAL: a WAL checkpoint (SQLite folding the
    // log back into the main file) can still briefly contend with a writer.
    // busy_timeout tells SQLite to retry for up to 5s before giving up,
    // instead of failing on the very first collision the way the code
    // above just did.
    check_sqlite(sqlite3_busy_timeout(db, 5000), db, "sqlite3_busy_timeout");

    static constexpr char kCreateTable[] =
        "CREATE TABLE IF NOT EXISTS scans ("
        "  seq INTEGER PRIMARY KEY,"
        "  stamp_ns INTEGER NOT NULL,"
        "  offset INTEGER NOT NULL,"
        "  length INTEGER NOT NULL,"
        "  sample_count INTEGER NOT NULL"
        ");";
    char* errmsg = nullptr;
    if (sqlite3_exec(db, kCreateTable, nullptr, nullptr, &errmsg) != SQLITE_OK) {
      const std::string message =
          std::string("ScanRecorder: CREATE TABLE failed: ") + (errmsg ? errmsg : "?");
      sqlite3_free(errmsg);
      throw std::runtime_error(message);
    }

    static constexpr char kInsert[] =
        "INSERT INTO scans (stamp_ns, offset, length, sample_count) VALUES (?, ?, ?, ?);";
    check_sqlite(sqlite3_prepare_v2(db, kInsert, -1, &insert_stmt, nullptr), db,
                 "sqlite3_prepare_v2(INSERT)");

    static constexpr char kCount[] = "SELECT COUNT(*) FROM scans;";
    sqlite3_stmt* count_stmt = nullptr;
    check_sqlite(sqlite3_prepare_v2(db, kCount, -1, &count_stmt, nullptr), db,
                 "sqlite3_prepare_v2(COUNT)");
    if (sqlite3_step(count_stmt) == SQLITE_ROW) {
      scan_count = static_cast<std::uint64_t>(sqlite3_column_int64(count_stmt, 0));
    }
    sqlite3_finalize(count_stmt);
    } catch (...) {
      close_resources();
      throw;
    }
  }

  void close_resources() noexcept {
    if (insert_stmt != nullptr) sqlite3_finalize(insert_stmt);
    insert_stmt = nullptr;
    if (db != nullptr) sqlite3_close(db);
    db = nullptr;
  }

  ~Impl() {
    close_resources();
    // log closes itself: std::ofstream's destructor flushes and closes.
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::ofstream log;
  sqlite3* db = nullptr;
  sqlite3_stmt* insert_stmt = nullptr;
  std::uint64_t scan_count = 0;

  // Reused across calls the same way to_proto's caller is encouraged to
  // reuse one LidarScan -- one allocation, not one per recorded scan.
  proto::LidarScan wire;
};

ScanRecorder::ScanRecorder(std::string log_path, std::string index_db_path)
    : impl_(std::make_unique<Impl>(log_path, index_db_path)) {}

ScanRecorder::~ScanRecorder() = default;
ScanRecorder::ScanRecorder(ScanRecorder&&) noexcept = default;
ScanRecorder& ScanRecorder::operator=(ScanRecorder&&) noexcept = default;

void ScanRecorder::record(const Scan& scan) {
  to_proto(scan, impl_->wire);
  const std::string bytes = impl_->wire.SerializeAsString();

  // Seek to end first: even though the stream was opened with ios::app
  // (every write lands at the end regardless), tellp() after an explicit
  // seek is what reliably reports that end offset back to us, which is the
  // number the index needs to record.
  impl_->log.seekp(0, std::ios::end);
  const std::int64_t offset = static_cast<std::int64_t>(impl_->log.tellp());

  write_length_prefix(impl_->log, static_cast<std::uint32_t>(bytes.size()));
  impl_->log.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  impl_->log.flush();
  if (!impl_->log.good()) {
    throw std::runtime_error("ScanRecorder: write to log failed");
  }

  sqlite3_reset(impl_->insert_stmt);
  sqlite3_bind_int64(impl_->insert_stmt, 1, scan.stamp_ns);
  sqlite3_bind_int64(impl_->insert_stmt, 2, offset);
  sqlite3_bind_int64(impl_->insert_stmt, 3, static_cast<std::int64_t>(bytes.size()));
  sqlite3_bind_int64(impl_->insert_stmt, 4, static_cast<std::int64_t>(scan.ranges.size()));
  check_sqlite(sqlite3_step(impl_->insert_stmt), impl_->db, "sqlite3_step(INSERT)");

  ++impl_->scan_count;
}

std::uint64_t ScanRecorder::count() const noexcept { return impl_->scan_count; }

}  // namespace lidar