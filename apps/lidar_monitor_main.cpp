// A single-command dashboard for Ram's own use, not the GUI team: watches
// the live push stream, the SQLite-backed recording, and the localhost
// query backup all at once, printing full sensor parameters as scans come
// in. Nothing here is new machinery -- it is ScanSubscriber, ScanReader,
// and ScanQueryClient, the exact three public types everyone else (the GUI
// team, the tests) already uses, just all three watched from one place so
// there is one command to run instead of three separate terminals to
// cross-check by eye.
//
// What each part proves, concretely:
//   [LIVE]   a scan actually arrived over the push stream (port 5556) --
//            confirms the publish side is alive and reachable.
//   [DB]     ScanReader's live row count against the SQLite index --
//            confirms scan_publisher_recorder_main is actually writing to
//            disk, not just publishing.
//   [QUERY]  a real request/reply round trip against the backup service on
//            localhost (port 5560) -- confirms the pull-based path the GUI
//            team depends on is genuinely answering, not just "the process
//            is running."
//
// Usage:
//   ./lidar_monitor_main <log_path> <db_path> [live_endpoint] [query_endpoint]
// Defaults: tcp://localhost:5556, tcp://localhost:5560
//
// Example (against scripts/start_lidar_service.sh's defaults):
//   ./lidar_monitor_main ~/lidar_data/dive.log ~/lidar_data/dive.db

#include "lidar/query_client.hpp"
#include "lidar/reader.hpp"
#include "lidar/scan.hpp"
#include "lidar/subscriber.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <string>

namespace {

std::atomic<bool> g_stop_requested{false};
void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }

// One line, every field a caller might want to sanity-check against the
// sensor's own spec sheet: point count, angular span, angular resolution,
// range limits, echo count. This is "sensor parameters and data from it"
// in one place, whichever of the three sources it came from.
void print_scan(const char* tag, const lidar::Scan& s) {
  std::printf(
      "[%s] frame_id=%-12s stamp_ns=%lld points=%-4zu "
      "angle=[%.3f, %.3f] rad (incr %.5f)  range=[%.2f, %.2f] m  echoes=%u\n",
      tag, s.frame_id.c_str(), static_cast<long long>(s.stamp_ns), s.ranges.size(),
      static_cast<double>(s.angle_min), static_cast<double>(s.angle_max),
      static_cast<double>(s.angle_increment), static_cast<double>(s.range_min),
      static_cast<double>(s.range_max), s.echo_count);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s <log_path> <db_path> [live_endpoint] [query_endpoint]\n"
                 "  defaults: live_endpoint=tcp://localhost:5556"
                 " query_endpoint=tcp://localhost:5560\n",
                 argv[0]);
    return 1;
  }

  const std::string log_path       = argv[1];
  const std::string db_path        = argv[2];
  const std::string live_endpoint  = argc > 3 ? argv[3] : "tcp://localhost:5556";
  const std::string query_endpoint = argc > 4 ? argv[4] : "tcp://localhost:5560";

  std::signal(SIGINT, handle_sigint);

  try {
    // Read-only from the start, same as scan_query_server_main -- this
    // never writes to the log or the index, only watches them.
    lidar::ScanReader reader(log_path, db_path);
    lidar::ScanSubscriber subscriber(live_endpoint, "lidar.scan");
    lidar::ScanQueryClient query_client(query_endpoint, /*timeout_ms=*/1000);

    std::printf(
        "lidar_monitor: watching db=%s live=%s query=%s (Ctrl+C to stop)\n\n",
        db_path.c_str(), live_endpoint.c_str(), query_endpoint.c_str());

    std::uint64_t live_received = 0;
    auto last_status = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    while (!g_stop_requested.load()) {
      lidar::Scan scan;
      if (subscriber.poll_scan(scan, /*timeout_ms=*/300)) {
        ++live_received;
        print_scan("LIVE", scan);
      }

      // A DB + query status line every ~2s -- more often would just be
      // noise, since the DB and the query backup don't change every 300ms
      // the way a live scan does.
      const auto now = std::chrono::steady_clock::now();
      if (now - last_status >= std::chrono::seconds(2)) {
        last_status = now;

        const std::uint64_t db_count = reader.count();

        const auto latest = query_client.latest();
        if (!query_client.ok()) {
          std::printf("[DB] %llu scan(s) indexed   [QUERY :5560] UNREACHABLE"
                      " -- is scan_query_server_main running?\n",
                      static_cast<unsigned long long>(db_count));
        } else if (!latest.has_value()) {
          std::printf("[DB] %llu scan(s) indexed   [QUERY :5560] reachable,"
                      " nothing recorded yet\n",
                      static_cast<unsigned long long>(db_count));
        } else {
          std::printf("[DB] %llu scan(s) indexed   ", static_cast<unsigned long long>(db_count));
          print_scan("QUERY :5560 latest", *latest);
        }

        if (live_received == 0) {
          std::printf("  (no live scans received yet on %s -- is"
                      " scan_publisher_recorder_main running and reachable?)\n",
                      live_endpoint.c_str());
        }
      }
    }

    std::printf("\nlidar_monitor: stopped (%llu live scan(s) seen this run)\n",
                static_cast<unsigned long long>(live_received));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "lidar_monitor: %s\n", e.what());
    return 1;
  }

  return 0;
}
