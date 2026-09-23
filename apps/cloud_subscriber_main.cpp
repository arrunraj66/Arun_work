#include "lidar/cloud_subscriber.hpp"
#include "lidar/point_cloud.hpp"

#include <cstdio>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  const std::string endpoint = argc > 1 ? argv[1] : "ipc:///tmp/lidar_cloud.sock";
  const std::string topic = argc > 2 ? argv[2] : "lidar.cloud";

  try {
    lidar::CloudSubscriber subscriber(endpoint, topic);
    std::printf("subscribed to %s on topic \"%s\" (Ctrl+C to stop) ...\n", endpoint.c_str(),
                topic.c_str());
    std::printf("note: clouds published before this connected are gone -- PUB/SUB drops to "
                "slow joiners by design\n");

    lidar::PointCloud cloud;
    std::uint64_t received = 0;
    std::uint64_t idle_polls = 0;

    while (true) {
      if (!subscriber.poll_cloud(cloud, /*timeout_ms=*/1000)) {
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
      std::printf("cloud #%llu: frame_id=%s stamp_ns=%lld points=%zu\n",
                  static_cast<unsigned long long>(received), cloud.frame_id.c_str(),
                  static_cast<long long>(cloud.stamp_ns), cloud.size());
      std::fflush(stdout);

      if (subscriber.malformed() > 0) {
        std::fprintf(stderr, "warning: %llu message(s) failed to parse -- wire format mismatch?\n",
                     static_cast<unsigned long long>(subscriber.malformed()));
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "cloud_subscriber_main: %s\n", e.what());
    return 1;
  }
}