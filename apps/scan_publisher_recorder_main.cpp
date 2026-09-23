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
// A NOTE ON CTRL+C: sick_scan_xd installs its OWN SIGINT handler as part of
// every connection attempt (inside SickScanApiCreate()). That handler is
// built for the vendor's own sick_generic_laser program, which owns the
// whole process -- on Ctrl+C it stops the sensor and prints its own
// "good bye", but it does NOT end our process. Left alone, our poll loop
// would just keep spinning on a now-dead connection forever.
//
// SickScanSource's own destructor already sends the sensor its proper stop
// commands (SickScanApiClose(), confirmed in sick_scan_source.cpp) -- that
// is already correct and needs no signal at all. So instead of relying on
// the vendor's handler, we install our own (last registration wins), and it
// does nothing but set a flag. The loop checks that flag and breaks;
// ordinary C++ destruction of `source` then sends the sensor its stop
// commands, every time, deterministically.
//
// A NOTE ON THE RECONNECT WATCHDOG (added after a real NIC link-flap on
// site caused SOPAS/TCP to stay up while UDP scan data silently stopped
// arriving for good): poll_scan() timing out occasionally is normal --
// the sensor briefly has nothing new. poll_scan() timing out over and
// over, AFTER this run has already received at least one real scan, is
// not normal -- it means the data path died mid-stream and, left alone,
// would just sit there printing "no scan within 1000 ms" forever until a
// human noticed and restarted the process by hand. The watchdog below
// turns that into something the service recovers from on its own: after
// kStallTimeoutCount consecutive timeouts post-first-scan (or
// kStartupTimeoutCount before this run has ever received one -- a longer,
// more patient threshold, so a fresh SOPAS handshake that's simply a
// couple of seconds slow isn't mistaken for a stall), it tears down and
// reconstructs `source` (a full reconnect -- new SOPAS handshake, new UDP
// receiver registration), exactly what a human restarting the process
// would have done, but without the multi-hour gap in the data while
// nobody was watching. Both thresholds fire eventually rather than
// hanging forever -- an earlier version of this watchdog only reconnected
// post-first-scan, and a real run showed exactly the gap that left open:
// SOPAS succeeding but zero UDP data from the very start of a run just
// counted timeouts forever with no recovery attempt at all.
//
// A NOTE ON CONNECT ATTEMPTS THAT NEVER RETURN: if the sensor is
// unreachable (unplugged, wrong IP, powered off) the moment a connection is
// attempted -- either the very first one, or one of the watchdog's reconnect
// attempts above -- SickScanSource's constructor can block inside the
// vendor SDK indefinitely, and because the vendor claims SIGINT for itself
// as PART OF that same blocked call, Ctrl+C cannot reliably reach us either
// while it's stuck. connect_with_stop_check() below is the fix: it runs the
// actual construction on a background thread and watches it with a timeout,
// so the thread that's checking for Ctrl+C is never the one that might be
// stuck. If a stop is requested while a connection attempt is still stuck,
// there is no safe way to cancel that background thread -- so instead of
// hanging waiting for it (the exact bug this exists to fix), the whole
// process exits immediately via std::_Exit(), which skips destructors and
// therefore never tries to wait for or clean up that stuck thread.
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <future>
#include <optional>
#include <string>
#include <thread>

namespace {

// A plain flag, not a class member: signal handlers in C++ may only touch
// objects with static storage duration and (for anything beyond
// sig_atomic_t) std::atomic is the portable way to do that safely. It is
// set from signal context and read from the ordinary flow of main() --
// nothing more is ever asked of it.
std::atomic<bool> g_stop_requested{false};

void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }

// After this many CONSECUTIVE poll_scan() timeouts, following at least one
// successful scan this run, the link is treated as stalled rather than
// merely quiet. 15 x the 1000 ms poll timeout = 15 s -- long enough that an
// ordinary brief gap between scans never trips it, short enough that a real
// stall gets caught and reconnected well within the window where the GUI
// team or a human diver-side operator would otherwise notice a dead feed.
constexpr int kStallTimeoutCount = 15;

// The ORIGINAL version of this watchdog only reconnected after data had
// already been flowing (ever_received == true) -- deliberately, so a sensor
// that was simply slow to boot, or genuinely misconfigured, would keep
// reporting that plainly instead of reconnect-looping before it had ever
// worked. Real hardware run exposed why that's not enough on its own: a
// run that starts with SOPAS succeeding but UDP data never arriving (the
// same "no scan within 1000 ms" symptom root-caused as an NIC link flap
// elsewhere) would then count consecutive timeouts forever with the
// watchdog never firing, because ever_received was still false -- the
// service just hangs, exactly the thing this watchdog exists to prevent.
// kStartupTimeoutCount is the fix: a separate, more patient threshold that
// applies BEFORE the first scan of this run, so a startup stall also gets a
// reconnect attempt eventually -- just a longer one, since a fresh SOPAS
// handshake and first packet can legitimately take a couple of seconds and
// shouldn't trigger a reconnect that a slightly slower boot would have
// resolved on its own. 30 x 1000 ms = 30 s.
constexpr int kStartupTimeoutCount = 30;

// Backoff between reconnect ATTEMPTS when the reconnect itself fails (e.g.
// the sensor is still unreachable, not just the previous connection gone
// stale). Sleeps in short slices so a Ctrl+C during a long backoff still
// stops promptly instead of waiting out the full delay.
void sleep_with_stop_check(std::chrono::milliseconds total) {
  const auto slice = std::chrono::milliseconds(100);
  auto remaining = total;
  while (remaining.count() > 0 && !g_stop_requested.load()) {
    const auto this_sleep = std::min(slice, remaining);
    std::this_thread::sleep_for(this_sleep);
    remaining -= this_sleep;
  }
}

// Attempts to construct a SickScanSource without ever blocking the calling
// thread for more than ~500ms at a time -- so the caller can keep checking
// g_stop_requested (and print progress) while a connection attempt that may
// never finish is still stuck on a background thread. See the file-level
// comment above for why this exists.
lidar::SickScanSource connect_with_stop_check(const lidar::SickScanSource::Config& cfg) {
  std::future<lidar::SickScanSource> fut =
      std::async(std::launch::async, [&cfg] { return lidar::SickScanSource(cfg); });

  
  int checks_waited = 0;
  while (true) {
    // Re-assert OUR handler on every check, not just once -- the vendor SDK
    // re-installs its own SIGINT handler as part of every connection
    // attempt, so this is us repeatedly re-winning that race rather than
    // losing it permanently the first time it happens.
    std::signal(SIGINT, handle_sigint);

    if (fut.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
      return fut.get();  // re-throws here if the constructor itself threw
    }

    ++checks_waited;
    if (checks_waited % 10 == 0) {  // every ~5s (10 x 500ms)
      std::printf("  ... still trying to connect to %s (%d s) ...\n", cfg.hostname.c_str(),
                  checks_waited / 2);
      std::fflush(stdout);
    }

    if (g_stop_requested.load()) {
      // No safe way to cancel the background thread -- it may still be
      // stuck inside the vendor SDK with no way out. _Exit() ends the whole
      // process immediately, skipping destructors, so nothing tries to wait
      // for or clean up that stuck thread on the way out.
      std::printf("\nCtrl+C received while still trying to connect -- exiting now.\n");
      std::fflush(stdout);
      std::_Exit(1);
    }
  }
}

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

    // std::optional, not a plain SickScanSource, precisely so the watchdog
    // below can destroy and reconstruct the connection in place -- a plain
    // stack object can't be re-constructed once it exists.
    std::printf("connecting to picoScan150 at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
                cfg.udp_receiver_ip.c_str());
    std::optional<lidar::SickScanSource> source;
    source = connect_with_stop_check(cfg);
    std::signal(SIGINT, handle_sigint);

    std::printf("connected. publishing + recording into %s / %s (Ctrl+C to stop) ...\n",
                log_path.c_str(), db_path.c_str());

    lidar::Scan scan;
    std::uint64_t recorded = 0;
    std::uint64_t published = 0;
    std::uint64_t last_reported = 0;
    std::uint64_t reconnects = 0;

    bool ever_received = false;
    int consecutive_timeouts = 0;

    while (!g_stop_requested.load()) {
      if (!source->poll_scan(scan, /*timeout_ms=*/1000)) {
        ++consecutive_timeouts;
        // Which threshold applies depends on whether this run has ever
        // received a scan: a startup stall gets more patience than a
        // stall that follows previously-working data (see the comment on
        // kStartupTimeoutCount above).
        const int threshold = ever_received ? kStallTimeoutCount : kStartupTimeoutCount;
        std::printf("  ... no scan within 1000 ms, still waiting (%d/%d before reconnect%s)\n",
                    consecutive_timeouts, threshold, ever_received ? "" : ", startup");
        std::fflush(stdout);

        if (consecutive_timeouts >= threshold) {
          std::printf(
              "  !!! watchdog: %d consecutive timeouts%s -- the link looks stalled, "
              "reconnecting to the sensor ...\n",
              consecutive_timeouts,
              ever_received ? " after previously receiving data" : " since startup");
          std::fflush(stdout);

          // Destroy first, then reconstruct: SickScanSource's destructor is
          // what sends the sensor its real stop command and releases the
          // vendor SDK's handle. Reconnecting without doing that first
          // would be asking sick_scan_xd to set up a second connection on
          // top of one it still thinks is live.
          source.reset();

          bool reconnected = false;
          while (!g_stop_requested.load() && !reconnected) {
            try {
              source = connect_with_stop_check(cfg);
              // Re-registered every time: see the comment above the first
              // std::signal() call -- the vendor constructor claims SIGINT
              // for itself again on every reconnect.
              std::signal(SIGINT, handle_sigint);
              reconnected = true;
              ++reconnects;
              consecutive_timeouts = 0;
              std::printf("  !!! watchdog: reconnected (reconnect #%llu)\n",
                          static_cast<unsigned long long>(reconnects));
              std::fflush(stdout);
            } catch (const std::exception& e) {
              std::fprintf(stderr,
                            "  !!! watchdog: reconnect attempt failed (%s) -- retrying in 5 s\n",
                            e.what());
              sleep_with_stop_check(std::chrono::milliseconds(5000));
            }
          }
        }
        continue;
      }

      ever_received = true;
      consecutive_timeouts = 0;

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
        std::printf("  ... %llu recorded, %llu published, %llu dropped, %llu reconnects, "
                    "last scan %zu points\n",
                    static_cast<unsigned long long>(recorded),
                    static_cast<unsigned long long>(published),
                    static_cast<unsigned long long>(publisher.dropped()),
                    static_cast<unsigned long long>(reconnects), scan.ranges.size());
        std::fflush(stdout);
      }
    }

    std::printf(
        "\nCtrl+C received -- stopping cleanly: %llu recorded, %llu published, %llu reconnects "
        "this run\n",
        static_cast<unsigned long long>(recorded), static_cast<unsigned long long>(published),
        static_cast<unsigned long long>(reconnects));
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