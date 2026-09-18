// Records real scans from the picoScan150 into the length-prefixed log and
// SQLite index -- the first program where Step 9 (the live sensor) and
// Step 6 (the recorder) are wired together.
//
// The only difference from record_sample_data is where the scans come from:
// that program invents them, this one polls a real IScanSource. Everything
// downstream -- ScanRecorder, the log format, the index, ScanReader,
// dump_scans -- is byte-for-byte the same machinery, unchanged. That is the
// payoff for having put IScanSource between the sensor and the rest of the
// library back in Step 3.
//
// Usage:
//   ./record_live_scans <launch_file> <sensor_ip> <this_machine_ip>
//                       [seconds] [log_path] [db_path]
//
// Defaults: 60 seconds, ./live_scans.log, ./live_scans.db
//
// Recording APPENDS: running it twice adds to the same files rather than
// starting over, exactly as ScanRecorder was built to do.

#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"
#include "sick_scan_source.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr,
                 "usage: %s <launch_file> <sensor_ip> <this_machine_ip> "
                 "[seconds] [log_path] [db_path]\n"
                 "  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
                 " 192.168.12.222 192.168.12.240 60\n"
                 "  (find this machine's IP on the sensor's subnet with: ip -4 addr show)\n",
                 argv[0], argv[0]);
    return 1;
  }

  lidar::SickScanSource::Config cfg;
  cfg.launch_file = argv[1];
  cfg.hostname = argv[2];
  cfg.udp_receiver_ip = argv[3];

  const int seconds = argc > 4 ? std::atoi(argv[4]) : 60;
  const std::string log_path = argc > 5 ? argv[5] : "./live_scans.log";
  const std::string db_path = argc > 6 ? argv[6] : "./live_scans.db";

  if (seconds <= 0) {
    std::fprintf(stderr, "record_live_scans: seconds must be positive (got %d)\n", seconds);
    return 1;
  }

  try {
    // Open the recorder BEFORE connecting the sensor. If the log or index
    // can't be opened we want to find out now, not after the sensor has
    // been started and is streaming data we have nowhere to put.
    lidar::ScanRecorder recorder(log_path, db_path);
    const std::uint64_t already_recorded = recorder.count();
    if (already_recorded > 0) {
      std::printf("%s already holds %llu scan(s); appending to it\n", db_path.c_str(),
                  static_cast<unsigned long long>(already_recorded));
    }

    std::printf("connecting to picoScan150 at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
                cfg.udp_receiver_ip.c_str());
    lidar::SickScanSource source(cfg);  // <-- the connect happens on this line
    std::printf("connected. recording for %d s into %s / %s (Ctrl+C to stop early) ...\n", seconds,
                log_path.c_str(), db_path.c_str());

    // steady_clock, not the sensor's timestamps: "record for 60 seconds"
    // means 60 seconds of OUR wall time. The sensor's own clock is not a
    // reliable stopwatch here -- its timestamp base switches part-way
    // through a run, which would make a duration computed from scan stamps
    // jump by years.
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::seconds(seconds);

    lidar::Scan scan;
    std::uint64_t recorded = 0;
    std::uint64_t timeouts = 0;
    int last_reported_second = -1;

    while (std::chrono::steady_clock::now() < deadline) {
      if (!source.poll_scan(scan, /*timeout_ms=*/500)) {
        ++timeouts;
        continue;  // nothing within 500 ms; loop round and check the deadline
      }
      recorder.record(scan);
      ++recorded;

      const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::steady_clock::now() - started)
                               .count();
      if (static_cast<int>(elapsed) != last_reported_second) {
        last_reported_second = static_cast<int>(elapsed);
        std::printf("  ... %llu scan(s) recorded (%lld s), last scan %zu points\n",
                    static_cast<unsigned long long>(recorded), static_cast<long long>(elapsed),
                    scan.ranges.size());
        std::fflush(stdout);
      }
    }

    std::printf("done: %llu scan(s) recorded this run, %llu now indexed in %s\n",
                static_cast<unsigned long long>(recorded),
                static_cast<unsigned long long>(recorder.count()), db_path.c_str());
    if (timeouts > 0) {
      std::printf("note: %llu poll(s) timed out with no scan -- normal at startup while the "
                  "sensor spins up, suspicious if it kept happening\n",
                  static_cast<unsigned long long>(timeouts));
    }
    if (recorded == 0) {
      std::fprintf(stderr, "record_live_scans: no scans were received -- nothing was recorded\n");
      return 1;
    }
    std::printf("inspect it with:\n  ./dump_scans %s %s 5\n", log_path.c_str(), db_path.c_str());
    // Both the source and the recorder are destroyed here: the sensor gets
    // its stop command, and the log and index are flushed and closed. No
    // shutdown code is written for either -- that is what their destructors
    // are for.
  } catch (const std::exception& e) {
    std::fprintf(stderr, "record_live_scans: %s\n", e.what());
    return 1;
  }

  return 0;
}