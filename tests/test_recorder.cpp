// Proves ScanRecorder's two halves agree with each other: record a few
// scans, then read them back using nothing but the index's own
// offset/length -- the same way a future ScanReader (Step 7) will -- and
// check the bytes deserialise into exactly what was recorded.

#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"
#include "scan_codec.hpp"

#include <sqlite3.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

lidar::Scan make_scan(int i) {
  lidar::Scan scan;
  scan.frame_id = "test_recorder";
  scan.stamp_ns = static_cast<std::int64_t>(i) * 100'000'000LL;
  scan.scan_time_ns = 100'000'000;
  scan.angle_min = -1.0F;
  scan.angle_max = 1.0F;
  scan.angle_increment = 0.5F;
  scan.range_min = 0.05F;
  scan.range_max = 20.0F;
  scan.echo_index = 0;
  scan.echo_count = 1;
  scan.ranges = {1.0F + static_cast<float>(i), 2.0F, 3.0F, 4.0F, 5.0F};
  return scan;
}

}  // namespace

int main() {
  const std::string log_path = "test_recorder_scans.log";
  const std::string db_path = "test_recorder_scans.db";
  std::remove(log_path.c_str());
  std::remove(db_path.c_str());

  constexpr int kNumScans = 3;
  {
    lidar::ScanRecorder recorder(log_path, db_path);
    for (int i = 0; i < kNumScans; ++i) {
      recorder.record(make_scan(i));
    }
    check(recorder.count() == kNumScans, "recorder.count() matches scans recorded");
  }  // recorder destroyed here -- log/db must both be safely closed and reopenable below

  sqlite3* db = nullptr;
  check(sqlite3_open(db_path.c_str(), &db) == SQLITE_OK, "index db reopens after recorder closes");

  sqlite3_stmt* stmt = nullptr;
  sqlite3_prepare_v2(db, "SELECT seq, stamp_ns, offset, length, sample_count FROM scans ORDER BY seq;",
                      -1, &stmt, nullptr);

  std::ifstream log(log_path, std::ios::binary);
  check(log.is_open(), "log file reopens after recorder closes");

  int rows_seen = 0;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const std::int64_t stamp_ns = sqlite3_column_int64(stmt, 1);
    const std::int64_t offset = sqlite3_column_int64(stmt, 2);
    const std::int64_t length = sqlite3_column_int64(stmt, 3);
    const std::int64_t sample_count = sqlite3_column_int64(stmt, 4);

    // Read exactly the way a reader must: seek to the index's offset, read
    // the 4-byte little-endian length prefix, then read that many bytes.
    log.seekg(offset);
    unsigned char prefix[4];
    log.read(reinterpret_cast<char*>(prefix), 4);
    const std::uint32_t declared_length = static_cast<std::uint32_t>(prefix[0]) |
                                           (static_cast<std::uint32_t>(prefix[1]) << 8) |
                                           (static_cast<std::uint32_t>(prefix[2]) << 16) |
                                           (static_cast<std::uint32_t>(prefix[3]) << 24);
    check(declared_length == static_cast<std::uint32_t>(length),
          "log's own length prefix matches the index's length column");

    std::string payload(declared_length, '\0');
    log.read(payload.data(), declared_length);
    check(static_cast<std::uint32_t>(log.gcount()) == declared_length,
          "read exactly as many payload bytes as declared");

    lidar::proto::LidarScan wire;
    check(wire.ParseFromString(payload), "payload parses back as a LidarScan");
    const lidar::Scan restored = lidar::from_proto(wire);

    const lidar::Scan expected = make_scan(rows_seen);
    check(restored.stamp_ns == expected.stamp_ns && restored.stamp_ns == stamp_ns,
          "restored stamp_ns matches both the original scan and the index");
    check(restored.ranges == expected.ranges, "restored ranges match the original scan");
    check(static_cast<std::int64_t>(restored.ranges.size()) == sample_count,
          "restored sample count matches the index's sample_count column");

    ++rows_seen;
  }
  check(rows_seen == kNumScans, "read back exactly as many rows as were recorded");

  sqlite3_finalize(stmt);
  sqlite3_close(db);

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}