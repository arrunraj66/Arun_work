// Step 7's test, straight from the lesson plan: "record 100 scans, read
// them all back, assert they are identical to what went in." Unlike
// test_recorder.cpp, this one only uses the PUBLIC API -- ScanRecorder and
// ScanReader -- because that's the whole point of Step 7: a caller who has
// never heard of protobuf or SQLite can still record and read scans back.

#include "lidar/reader.hpp"
#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

// Same generator shape as test_recorder.cpp's make_scan, so a reader and
// writer bug can't accidentally cancel each other out between the two
// tests -- deliberately not shared code.
lidar::Scan make_scan(int i) {
  lidar::Scan scan;
  scan.frame_id = "test_reader";
  scan.stamp_ns = static_cast<std::int64_t>(i) * 100'000'000LL;  // 10 Hz
  scan.scan_time_ns = 100'000'000;
  scan.time_increment_ns = 100'000'000 / 360;
  scan.angle_min = -3.14F;
  scan.angle_max = 3.14F;
  scan.angle_increment = 0.01745F;
  scan.range_min = 0.05F;
  scan.range_max = 20.0F;
  scan.echo_index = 0;
  scan.echo_count = 1;
  scan.ranges = {1.0F + static_cast<float>(i) * 0.01F, 2.0F, 3.0F, 4.0F, 5.0F};
  scan.intensities = {10.0F, 20.0F, 30.0F, 40.0F, 50.0F};
  return scan;
}

bool same_scan(const lidar::Scan& a, const lidar::Scan& b) {
  return a.frame_id == b.frame_id && a.stamp_ns == b.stamp_ns && a.scan_time_ns == b.scan_time_ns &&
         a.time_increment_ns == b.time_increment_ns && a.angle_min == b.angle_min &&
         a.angle_max == b.angle_max && a.angle_increment == b.angle_increment &&
         a.range_min == b.range_min && a.range_max == b.range_max && a.echo_index == b.echo_index &&
         a.echo_count == b.echo_count && a.ranges == b.ranges && a.intensities == b.intensities;
}

}  // namespace

int main() {
  const std::string log_path = "test_reader_scans.log";
  const std::string db_path = "test_reader_scans.db";
  std::remove(log_path.c_str());
  std::remove(db_path.c_str());

  constexpr int kNumScans = 100;
  {
    lidar::ScanRecorder recorder(log_path, db_path);
    for (int i = 0; i < kNumScans; ++i) {
      recorder.record(make_scan(i));
    }
  }  // recorder destroyed -- both files closed before the reader opens them

  lidar::ScanReader reader(log_path, db_path);
  check(reader.count() == static_cast<std::uint64_t>(kNumScans),
        "reader.count() matches the number of scans recorded");

  // get(i): every single scan, in order, byte-for-byte the same as what
  // went in. This is the round trip the lesson plan calls "the test that
  // matters."
  bool all_match = true;
  for (int i = 0; i < kNumScans; ++i) {
    const lidar::Scan restored = reader.get(static_cast<std::uint64_t>(i));
    if (!same_scan(restored, make_scan(i))) {
      all_match = false;
      std::printf("  mismatch at index %d\n", i);
    }
  }
  check(all_match, "get(i) for every i in [0, 100) matches what was recorded");

  // get() past the end must throw, not read garbage or crash.
  bool threw = false;
  try {
    [[maybe_unused]] const lidar::Scan unused = reader.get(static_cast<std::uint64_t>(kNumScans));
  } catch (const std::out_of_range&) {
    threw = true;
  }
  check(threw, "get(count()) throws std::out_of_range instead of reading past the end");

  // find_by_time_range: scans 10..19 have stamp_ns 1.0e9 .. 1.9e9 (10 Hz,
  // so consecutive scans are 100ms apart) -- ask for exactly that window
  // and check we get exactly those 10 scans, in order, nothing more.
  const auto range = reader.find_by_time_range(1'000'000'000LL, 1'900'000'000LL);
  check(range.size() == 10, "find_by_time_range(1.0s, 1.9s) returns exactly 10 scans");
  bool range_matches = range.size() == 10;
  if (range_matches) {
    for (int i = 0; i < 10; ++i) {
      if (!same_scan(range[static_cast<std::size_t>(i)], make_scan(i + 10))) {
        range_matches = false;
        std::printf("  range mismatch at position %d (expected scan %d)\n", i, i + 10);
      }
    }
  }
  check(range_matches, "find_by_time_range results are exactly scans 10..19, in order");

  // A window that matches nothing must come back empty, not throw.
  const auto empty_range = reader.find_by_time_range(-1'000'000'000LL, -1LL);
  check(empty_range.empty(), "find_by_time_range with no matches returns an empty vector");

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}