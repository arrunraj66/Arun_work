// Step 6's runnable half: read the real multiScan136 and publish every
// cloud onto the network. Run this in one terminal and cloud_subscriber_main
// in another. Same shape as scan_publisher_main.cpp -- the loop is short on
// purpose, everything hard is behind SickCloudSource, CloudPublisher, and
// the codec.
//
// Usage:
//   ./cloud_publisher_main <launch_file> <sensor_ip> <this_machine_ip>
//                          [endpoint] [topic] [udp_port] [imu_udp_port]
// Defaults: ipc:///tmp/lidar_cloud.sock, lidar.cloud, 0 (launch file
// default), 0 (launch file default)
//
// To publish to other machines, bind a TCP endpoint instead. To run
// alongside a 2D picoScan150 at the same time, pass DISTINCT udp_port /
// imu_udp_port values (e.g. 2125 7513) so the two sensors' drivers don't
// fight over the same UDP ports:
//   ./cloud_publisher_main ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch
//       192.168.12.223 192.168.12.240 "tcp://*:5580" lidar.cloud 2125 7513

#include "lidar/cloud_publisher.hpp"
#include "lidar/point_cloud.hpp"
#include "sick_cloud_source.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <exception>
#include <string>

namespace {
std::atomic<bool> g_stop_requested{false};
void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr,
                 "usage: %s <launch_file> <sensor_ip> <this_machine_ip> "
                 "[endpoint] [topic] [udp_port] [imu_udp_port]\n"
                 "  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch"
                 " 192.168.12.223 192.168.12.240\n",
                 argv[0], argv[0]);
    return 1;
  }

  lidar::SickCloudSource::Config cfg;
  cfg.launch_file = argv[1];
  cfg.hostname = argv[2];
  cfg.udp_receiver_ip = argv[3];

  const std::string endpoint = argc > 4 ? argv[4] : "ipc:///tmp/lidar_cloud.sock";
  const std::string topic = argc > 5 ? argv[5] : "lidar.cloud";
  cfg.udp_port = argc > 6 ? std::atoi(argv[6]) : 0;
  cfg.imu_udp_port = argc > 7 ? std::atoi(argv[7]) : 0;

  try {
    // Bind before connecting to the sensor -- same reasoning as
    // scan_publisher_main: if the endpoint is unusable we want to know
    // before the sensor is streaming.
    lidar::CloudPublisher publisher(endpoint, topic);
    std::printf("publishing on %s, topic \"%s\"\n", endpoint.c_str(), topic.c_str());

    std::printf("connecting to 3D lidar at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
                cfg.udp_receiver_ip.c_str());
    lidar::SickCloudSource source(cfg);

    // Installed AFTER the connect -- see sick_cloud_source_demo.cpp for why.
    std::signal(SIGINT, handle_sigint);

    std::printf("connected. publishing clouds (Ctrl+C to stop) ...\n");

    lidar::PointCloud cloud;
    std::uint64_t published = 0;
    std::uint64_t last_reported = 0;

    while (!g_stop_requested.load()) {
      if (!source.poll_cloud(cloud, /*timeout_ms=*/1000)) {
        std::printf("  ... no cloud within 1000 ms, still waiting\n");
        std::fflush(stdout);
        continue;
      }
      if (publisher.publish(cloud)) {
        ++published;
      }
      // publish() returning false means the send queue was full and the
      // cloud was dropped -- a subscriber is too slow. Same "count it,
      // don't stop for it" outcome as scan_publisher_main.

      if (published - last_reported >= 20) {  // ~1 s at ~20 Hz
        last_reported = published;
        std::printf("  ... %llu published, %llu dropped, last cloud %zu points\n",
                    static_cast<unsigned long long>(published),
                    static_cast<unsigned long long>(publisher.dropped()), cloud.size());
        std::fflush(stdout);
      }
    }
    std::printf("\ncloud_publisher_main: stopping -- %llu published, %llu dropped\n",
                static_cast<unsigned long long>(published),
                static_cast<unsigned long long>(publisher.dropped()));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cloud_publisher_main: %s\n", e.what());
    return 1;
  }
}
