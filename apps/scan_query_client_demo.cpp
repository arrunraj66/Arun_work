// The backup path's client half, for the GUI team (or anyone) to try by
// hand. Two commands:
//
//   ./scan_query_client_demo <endpoint> latest
//   ./scan_query_client_demo <endpoint> range <start_ns> <end_ns>
//
// e.g. ./scan_query_client_demo tcp://192.168.12.240:5560 latest

#include "lidar/query_client.hpp"
#include "lidar/scan.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

void print_scan(const lidar::Scan& s) {
  std::printf("frame_id=%s stamp_ns=%lld points=%zu angle=[%.3f, %.3f] rad\n",
              s.frame_id.c_str(), static_cast<long long>(s.stamp_ns), s.ranges.size(),
              static_cast<double>(s.angle_min), static_cast<double>(s.angle_max));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s <endpoint> latest\n"
                 "       %s <endpoint> range <start_ns> <end_ns>\n",
                 argv[0], argv[0]);
    return 1;
  }

  const std::string endpoint = argv[1];
  const std::string command  = argv[2];

  lidar::ScanQueryClient client(endpoint, /*timeout_ms=*/3000);

  if (command == "latest") {
    const auto scan = client.latest();
    if (!client.ok()) {
      std::fprintf(stderr, "no reply from %s -- is scan_query_server_main running there?\n",
                   endpoint.c_str());
      return 1;
    }
    if (!scan.has_value()) {
      std::printf("server reached, but nothing has been recorded yet\n");
      return 0;
    }
    print_scan(*scan);
    return 0;
  }

  if (command == "range" && argc >= 5) {
    const std::int64_t start_ns = std::atoll(argv[3]);
    const std::int64_t end_ns   = std::atoll(argv[4]);
    const auto scans = client.range(start_ns, end_ns);
    if (!client.ok()) {
      std::fprintf(stderr, "no reply from %s -- is scan_query_server_main running there?\n",
                   endpoint.c_str());
      return 1;
    }
    std::printf("%zu scan(s) in range\n", scans.size());
    for (const auto& s : scans) print_scan(s);
    return 0;
  }

  std::fprintf(stderr, "unrecognised command '%s'\n", command.c_str());
  return 1;
}
