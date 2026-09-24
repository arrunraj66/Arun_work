// Step 5's runnable half: connect to a ScanPublisher and print what
// arrives. Run this in one terminal and scan_publisher_main in another.
//
// Deliberately has no idea where the scans originate. They might be coming
// off a picoScan150 in the next room, off a recorded log being replayed, or
// from a different machine entirely -- this program asks an IScanSource for
// scans and prints them, and that is the whole of its knowledge.
//
// Usage:
//   ./scan_subscriber_main [endpoint] [topic]
// Defaults: ipc:///tmp/lidar.sock, lidar.scan
//
// Across machines use the publisher's address, e.g.
//   ./scan_subscriber_main tcp://192.168.12.240:5556

#include "lidar/scan.hpp"
#include "lidar/subscriber.hpp"

#include <cstdio>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  const std::string endpoint = argc > 1 ? argv[1] : "ipc:///tmp/lidar.sock";
  const std::string topic = argc > 2 ? argv[2] : "lidar.scan";

  try {
    lidar::ScanSubscriber subscriber(endpoint, topic);
    std::printf("subscribed to %s on topic \"%s\" (Ctrl+C to stop) ...\n", endpoint.c_str(),
                topic.c_str());
    std::printf("note: scans published before this connected are gone -- PUB/SUB drops to "
                "slow joiners by design\n");

    lidar::Scan scan;
    std::uint64_t received = 0;
    std::uint64_t idle_polls = 0;

    while (true) {
      if (!subscriber.poll_scan(scan, /*timeout_ms=*/1000)) {
        ++idle_polls;
        if (idle_polls % 5 == 0) {
          std::printf("  ... nothing for %llu s (is the publisher running?)\n",
                      static_cast<unsigned long long>(idle_polls));
          std::fflush(stdout);
        }
        continue;
      }
      idle_polls = 0;
      ++received;
      std::printf("scan #%llu: frame_id=%s stamp_ns=%lld points=%zu angle=[%.3f, %.3f] rad  "
                  "first=%.3f m last=%.3f m\n",
                  static_cast<unsigned long long>(received), scan.frame_id.c_str(),
                  static_cast<long long>(scan.stamp_ns), scan.ranges.size(),
                  static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max),
                  scan.ranges.empty() ? 0.0 : static_cast<double>(scan.ranges.front()),
                  scan.ranges.empty() ? 0.0 : static_cast<double>(scan.ranges.back()));
      std::fflush(stdout);

      if (subscriber.malformed() > 0) {
        std::fprintf(stderr, "warning: %llu message(s) failed to parse -- wire format mismatch?\n",
                     static_cast<unsigned long long>(subscriber.malformed()));
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "scan_subscriber_main: %s\n", e.what());
    return 1;
  }
}
