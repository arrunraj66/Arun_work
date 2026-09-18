// The backup path's server half: answers "latest" and "range" requests
// straight from a recording. Needs no sensor SDK at all -- it only reads the
// log+index ScanRecorder produces, whether that recorder is still running
// (recording live) or finished long ago (a replay of an old dive).
//
// Run this alongside scan_publisher_main against the same log/db files while
// recording live, so the GUI team has a pull-based fallback available the
// whole time the live push stream is up -- not just after the fact.
//
// Usage:
//   ./scan_query_server_main <log_path> <db_path> [endpoint]
// Default endpoint: tcp://*:5560

#include "lidar/query_server.hpp"

#include <cstdio>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <log_path> <db_path> [endpoint]\n", argv[0]);
    return 1;
  }

  const std::string log_path = argv[1];
  const std::string db_path  = argv[2];
  const std::string endpoint = argc > 3 ? argv[3] : "tcp://*:5560";

  try {
    lidar::ScanQueryServer server(log_path, db_path, endpoint);
    std::printf("answering queries on %s from %s / %s (Ctrl+C to stop)\n", endpoint.c_str(),
                log_path.c_str(), db_path.c_str());

    std::uint64_t last_reported = 0;
    while (true) {
      const bool served = server.serve_one(/*timeout_ms=*/1000);
      if (served && server.served() - last_reported >= 10) {
        last_reported = server.served();
        std::printf("  ... %llu requests answered so far\n",
                    static_cast<unsigned long long>(server.served()));
        std::fflush(stdout);
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "scan_query_server_main: %s\n", e.what());
    return 1;
  }
}