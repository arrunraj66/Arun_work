// The actual "node" — a real program with a real main(), the same shape as
// a ROS node, just without ROS. SickScanSource by itself is only a class
// sitting in a library; nothing runs until some program constructs one.
// This is that program, doing nothing but proving the sensor connects,
// streams, and disconnects.
//
// What it does, top to bottom:
//   1. build a SickScanSource        -- this line IS the connect
//   2. poll it in a loop, print each scan's summary
//   3. let it go out of scope        -- this IS the disconnect
//
// Nothing about ZeroMQ, the publisher, or the recorder is here yet -- this
// is deliberately the smallest possible program that proves SickScanSource
// itself works, before it's wired into anything bigger.
//
// Usage:
//   ./sick_scan_source_demo <launch_file> <sensor_ip> <this_machine_ip>
//
// Example (the combination verified working against the real unit):
//   ./sick_scan_source_demo ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch
//                           192.168.12.222 192.168.12.240
//
// All three arguments are required. <this_machine_ip> is the address the
// sensor pushes UDP scan data to -- find it with `ip -4 addr show`.

#include "sick_scan_source.hpp"

#include <cstdio>
#include <exception>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr,
                 "usage: %s <launch_file> <sensor_ip> <this_machine_ip>\n"
                 "  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_picoscan.launch"
                 " 192.168.12.222 192.168.12.240\n"
                 "  (find this machine's IP on the sensor's subnet with: ip -4 addr show)\n",
                 argv[0], argv[0]);
    return 1;
  }

  lidar::SickScanSource::Config cfg;
  cfg.launch_file = argv[1];
  cfg.hostname = argv[2];
  cfg.udp_receiver_ip = argv[3];

  std::printf("connecting to picoScan150 at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
              cfg.udp_receiver_ip.c_str());

  try {
    lidar::SickScanSource source(cfg);  // <-- the connect happens on this line

    std::printf("connected. printing scans (Ctrl+C to stop) ...\n");

    lidar::Scan scan;
    int scan_count = 0;
    while (true) {
      if (!source.poll_scan(scan, /*timeout_ms=*/1000)) {
        std::printf("  ... no scan within 1000 ms, still waiting\n");
        continue;
      }
      ++scan_count;
      std::printf("scan #%d: frame_id=%s stamp_ns=%lld points=%zu scan_time_ms=%.1f\n",
                  scan_count, scan.frame_id.c_str(), static_cast<long long>(scan.stamp_ns),
                  scan.ranges.size(),
                  static_cast<double>(scan.scan_time_ns) / 1.0e6);
      if (!scan.ranges.empty()) {
        std::printf("  angle=[%.3f, %.3f] rad  increment=%.5f  first range=%.3f m  last=%.3f m\n",
                    static_cast<double>(scan.angle_min), static_cast<double>(scan.angle_max),
                    static_cast<double>(scan.angle_increment),
                    static_cast<double>(scan.ranges.front()),
                    static_cast<double>(scan.ranges.back()));
      }
    }
    // Unreachable today (the loop above only exits via Ctrl+C), but
    // source's destructor runs the automatic disconnect the moment this
    // scope ends -- nothing needs to be added here to make that happen.
  } catch (const std::exception& e) {
    // The constructor throws if the configuration is incomplete or the
    // connect/enable handshake fails -- see SickScanSource's header.
    // Catching it here turns that into one readable line instead of the
    // program aborting.
    std::fprintf(stderr, "failed to connect: %s\n", e.what());
    return 1;
  }
}
