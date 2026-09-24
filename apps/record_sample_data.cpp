// Generates a fixed duration of deterministic, plausible-looking scans and
// records them with ScanRecorder -- sample data for exercising the log +
// SQLite index (and, later, the reader) without needing a live sensor or
// network connectivity at all. Deliberately not IScanSource: this is a
// one-off data-generation tool, not a source the publisher would ever use.
//
// Usage: ./record_sample_data [seconds] [rate_hz] [log_path] [db_path]
// Defaults: 60 seconds, 10 Hz, ./sample_scans.log, ./sample_scans.db

#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

constexpr int kSamplesPerScan = 360;
constexpr float kRangeMin = 0.05F;
constexpr float kRangeMax = 20.0F;
constexpr float kPi = 3.14159265358979323846F;

// A deterministic, plausible-looking scan: a wall at ~2m with a gentle
// four-lobed ripple, slowly rotating scan to scan so successive scans
// aren't bit-for-bit identical. No randomness needed -- reproducibility
// matters more than realism for sample/reference data.
lidar::Scan make_sample_scan(int scan_index, std::int64_t stamp_ns,
                              std::int64_t scan_time_ns) {
  lidar::Scan scan;
  scan.frame_id = "sample_data";
  scan.stamp_ns = stamp_ns;
  scan.scan_time_ns = scan_time_ns;
  scan.time_increment_ns = scan_time_ns / kSamplesPerScan;
  scan.angle_min = -kPi;
  scan.angle_max = kPi - (2.0F * kPi / static_cast<float>(kSamplesPerScan));
  scan.angle_increment = 2.0F * kPi / static_cast<float>(kSamplesPerScan);
  scan.range_min = kRangeMin;
  scan.range_max = kRangeMax;
  scan.echo_index = 0;
  scan.echo_count = 1;

  scan.ranges.reserve(kSamplesPerScan);
  const float drift = static_cast<float>(scan_index) * 0.01F;
  for (int i = 0; i < kSamplesPerScan; ++i) {
    const float angle = lidar::angle_of(scan, static_cast<std::size_t>(i));
    const float range = 2.0F + 0.3F * std::sin(4.0F * angle + drift);
    scan.ranges.push_back(range);
  }
  return scan;
}

}  // namespace

int main(int argc, char** argv) {
  const int seconds = argc > 1 ? std::atoi(argv[1]) : 60;
  const int rate_hz = argc > 2 ? std::atoi(argv[2]) : 10;
  const std::string log_path = argc > 3 ? argv[3] : "./sample_scans.log";
  const std::string db_path = argc > 4 ? argv[4] : "./sample_scans.db";

  if (seconds <= 0 || rate_hz <= 0) {
    std::fprintf(stderr, "seconds and rate_hz must both be positive\n");
    return 1;
  }

  const int num_scans = seconds * rate_hz;
  const std::int64_t scan_time_ns = 1'000'000'000LL / rate_hz;

  std::printf("recording %d scans (%d seconds at %d Hz) to %s / %s ...\n", num_scans,
              seconds, rate_hz, log_path.c_str(), db_path.c_str());

  lidar::ScanRecorder recorder(log_path, db_path);

  for (int i = 0; i < num_scans; ++i) {
    const std::int64_t stamp_ns = static_cast<std::int64_t>(i) * scan_time_ns;
    recorder.record(make_sample_scan(i, stamp_ns, scan_time_ns));
    if ((i + 1) % rate_hz == 0) {
      std::printf("  ... %d/%d scans recorded (%d s)\n", i + 1, num_scans, (i + 1) / rate_hz);
    }
  }

  std::printf("done: %llu scans now indexed in %s\n",
              static_cast<unsigned long long>(recorder.count()), db_path.c_str());
  return 0;
}
