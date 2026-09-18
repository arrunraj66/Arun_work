// The real handoff program: one sensor connection, two things happen to
// every scan -- it goes out live over ZeroMQ (for scan_subscriber_main /
// the GUI's live view) AND it is appended to the log+index (so
// scan_query_server_main always has something current to answer from).
//
// This replaces running scan_publisher_main and record_live_scans
// separately. They cannot safely run at the same time against the real
// sensor anyway -- each does its own SOPAS handshake and tells the sensor
// where to send UDP scan data, so two independent processes fighting over
// that configuration is the same class of problem as two processes fighting
// over one socket. One connection, one owner, two outputs.
//
// Unlike record_live_scans, this does not stop after N seconds -- a
// standing service should keep running until you stop it.
//
// A NOTE ON CTRL+C: sick_scan_xd installs its OWN SIGINT handler the moment
// SickScanSource connects (inside SickScanApiCreate()). That handler is
// built for the vendor's own sick_generic_laser program, which owns the
// whole process -- on Ctrl+C it stops the sensor and prints its own
// "good bye", but it does NOT end our process. Left alone, our poll loop
// would just keep spinning on a now-dead connection forever.
//
// SickScanSource's own destructor already sends the sensor its proper stop
// commands (SickScanApiClose(), confirmed in sick_scan_source.cpp) -- that
// is already correct and needs no signal at all. So instead of relying on
// the vendor's handler, we install our own AFTER constructing the source
// (last registration wins), and it does nothing but set a flag. The loop
// checks that flag and breaks; ordinary C++ destruction of `source` then
// sends the sensor its stop commands, every time, deterministically.
//
// Usage:
//   ./scan_publisher_recorder_main <launch_file> <sensor_ip> <this_machine_ip>
//                                   <log_path> <db_path> [endpoint] [topic]
// Defaults: endpoint tcp://*:5556, topic lidar.scan
//
// Example:
//   ./scan_publisher_recorder_main ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch
//       192.168.12.222 192.168.12.240
//       ~/lidar_data/dive.log ~/lidar_data/dive.db "tcp://*:5556"
//
// Recording APPENDS, same as record_live_scans -- running this again after a
// restart adds to the same files rather than starting over.

#include "lidar/publisher.hpp"
#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"
#include "sick_scan_source.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <exception>
#include <string>

namespace {

// A plain flag, not a class member: signal handlers in C++ may only touch
// objects with static storage duration and (for anything beyond
// sig_atomic_t) std::atomic is the portable way to do that safely. It is
// set from signal context and read from the ordinary flow of main() --
// nothing more is ever asked of it.
std::atomic<bool> g_stop_requested{false};

void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }

}  // namespace

int main(int argc, char** argv) {
  if (argc < 6) {
    std::fprintf(stderr,
                 "usage: %s <launch_file> <sensor_ip> <this_machine_ip> "
                 "<log_path> <db_path> [endpoint] [topic]\n"
                 "  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
                 " 192.168.12.222 192.168.12.240"
                 " ~/lidar_data/dive.log ~/lidar_data/dive.db \"tcp://*:5556\"\n",
                 argv[0], argv[0]);
    return 1;
  }

  lidar::SickScanSource::Config cfg;
  cfg.launch_file = argv[1];
  cfg.hostname = argv[2];
  cfg.udp_receiver_ip = argv[3];

  const std::string log_path = argv[4];
  const std::string db_path  = argv[5];
  const std::string endpoint = argc > 6 ? argv[6] : "tcp://*:5556";
  const std::string topic    = argc > 7 ? argv[7] : "lidar.scan";

  try {
    // Same ordering discipline as every constructor in this library: open
    // everything that can fail cheaply -- the recorder's files, the
    // publisher's bind -- before touching the sensor, so a bad path or a
    // busy port is reported before the sensor has been told to start
    // streaming into nothing.
    lidar::ScanRecorder recorder(log_path, db_path);
    const std::uint64_t already_recorded = recorder.count();
    if (already_recorded > 0) {
      std::printf("%s already holds %llu scan(s); appending to it\n", db_path.c_str(),
                  static_cast<unsigned long long>(already_recorded));
    }

    lidar::ScanPublisher publisher(endpoint, topic);
    std::printf("publishing on %s, topic \"%s\"\n", endpoint.c_str(), topic.c_str());

    std::printf("connecting to picoScan150 at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
                cfg.udp_receiver_ip.c_str());
    lidar::SickScanSource source(cfg);

    // Must come AFTER constructing `source`: its constructor is what
    // installs sick_scan_xd's own SIGINT handler, so registering ours here
    // makes ours the one that actually runs on Ctrl+C.
    std::signal(SIGINT, handle_sigint);

    std::printf("connected. publishing + recording into %s / %s (Ctrl+C to stop) ...\n",
                log_path.c_str(), db_path.c_str());

    lidar::Scan scan;
    std::uint64_t recorded = 0;
    std::uint64_t published = 0;
    std::uint64_t last_reported = 0;

    while (!g_stop_requested.load()) {
      if (!source.poll_scan(scan, /*timeout_ms=*/1000)) {
        std::printf("  ... no scan within 1000 ms, still waiting\n");
        std::fflush(stdout);
        continue;
      }

      // Record first: the log is the thing scan_query_server_main and any
      // later replay depend on, so it is the one output that must never be
      // skipped. Publishing after is best-effort by design (publish() can
      // legitimately drop under a slow subscriber); recording is not.
      recorder.record(scan);
      ++recorded;

      if (publisher.publish(scan)) {
        ++published;
      }

      if (recorded - last_reported >= 25) {  // ~1 s at 25 Hz
        last_reported = recorded;
        std::printf("  ... %llu recorded, %llu published, %llu dropped, last scan %zu points\n",
                    static_cast<unsigned long long>(recorded),
                    static_cast<unsigned long long>(published),
                    static_cast<unsigned long long>(publisher.dropped()), scan.ranges.size());
        std::fflush(stdout);
      }
    }

    std::printf("\nCtrl+C received -- stopping cleanly: %llu recorded, %llu published this run\n",
                static_cast<unsigned long long>(recorded),
                static_cast<unsigned long long>(published));
    // `source`, `publisher`, and `recorder` are destroyed here, in reverse
    // declaration order: the sensor gets its real stop command via
    // SickScanSource's destructor, and the log+index are flushed and
    // closed via ScanRecorder's -- no shutdown code needed for either,
    // same reasoning as record_live_scans.
  } catch (const std::exception& e) {
    std::fprintf(stderr, "scan_publisher_recorder_main: %s\n", e.what());
    return 1;
  }

  return 0;
}