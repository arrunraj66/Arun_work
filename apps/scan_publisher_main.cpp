// Step 4's runnable half: read the real picoScan150 and publish every scan
// onto the network. Run this in one terminal and scan_subscriber_main in
// another.
//
// The loop is four lines, and that is the point. Everything hard -- the
// sensor, the wire format, the socket -- is behind SickScanSource,
// ScanPublisher, and the codec. This program only decides that scans
// should go from the one to the other.
//
// Usage:
//   ./scan_publisher_main <launch_file> <sensor_ip> <this_machine_ip>
//                         [endpoint] [topic]
// Defaults: ipc:///tmp/lidar.sock, lidar.scan
//
// To publish to other machines, bind a TCP endpoint instead:
//   ./scan_publisher_main <launch> 192.168.12.222 192.168.12.240 "tcp://*:5556"

#include "lidar/publisher.hpp"
#include "lidar/scan.hpp"
#include "sick_scan_source.hpp"

#include <cstdio>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr,
                 "usage: %s <launch_file> <sensor_ip> <this_machine_ip> [endpoint] [topic]\n"
                 "  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
                 " 192.168.12.222 192.168.12.240\n",
                 argv[0], argv[0]);
    return 1;
  }

  lidar::SickScanSource::Config cfg;
  cfg.launch_file = argv[1];
  cfg.hostname = argv[2];
  cfg.udp_receiver_ip = argv[3];

  const std::string endpoint = argc > 4 ? argv[4] : "ipc:///tmp/lidar.sock";
  const std::string topic = argc > 5 ? argv[5] : "lidar.scan";

  try {
    // Bind before connecting to the sensor: if the endpoint is unusable we
    // want to know before the sensor is streaming.
    lidar::ScanPublisher publisher(endpoint, topic);
    std::printf("publishing on %s, topic \"%s\"\n", endpoint.c_str(), topic.c_str());

    std::printf("connecting to picoScan150 at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
                cfg.udp_receiver_ip.c_str());
    lidar::SickScanSource source(cfg);
    std::printf("connected. publishing scans (Ctrl+C to stop) ...\n");

    lidar::Scan scan;
    std::uint64_t published = 0;
    std::uint64_t last_reported = 0;

    while (true) {
      if (!source.poll_scan(scan, /*timeout_ms=*/1000)) {
        std::printf("  ... no scan within 1000 ms, still waiting\n");
        std::fflush(stdout);
        continue;
      }
      if (publisher.publish(scan)) {
        ++published;
      }
      // publish() returning false means the send queue was full and the
      // scan was dropped -- a subscriber is too slow. For live sensor data
      // that is the correct outcome, so it is counted, not treated as an
      // error worth stopping for.

      if (published - last_reported >= 25) {  // ~1 s at 25 Hz
        last_reported = published;
        std::printf("  ... %llu published, %llu dropped, last scan %zu points\n",
                    static_cast<unsigned long long>(published),
                    static_cast<unsigned long long>(publisher.dropped()), scan.ranges.size());
        std::fflush(stdout);
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "scan_publisher_main: %s\n", e.what());
    return 1;
  }
}