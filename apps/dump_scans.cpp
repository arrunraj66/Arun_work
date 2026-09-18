// A small command-line tool built entirely on ScanReader's PUBLIC API --
// answers "what's actually in this log+index pair" without needing sqlite3
// or a hex editor. This is the human-readable counterpart to looking at the
// SQLite index directly: the index only has seq/stamp_ns/offset/length, this
// prints the actual scan a row points at.
//
// Usage: ./dump_scans <log_path> <db_path> [max_scans]
// Defaults: prints every scan. Pass a number to cap it (useful once a log
// has thousands of scans in it).

#include "lidar/reader.hpp"
#include "lidar/scan.hpp"

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <log_path> <db_path> [max_scans]\n", argv[0]);
    return 1;
  }

  const std::string log_path = argv[1];
  const std::string db_path = argv[2];
  const std::uint64_t max_scans = argc > 3 ? static_cast<std::uint64_t>(std::atoll(argv[3])) : 0;

  try {
    lidar::ScanReader reader(log_path, db_path);
    const std::uint64_t total = reader.count();
    std::printf("%s / %s: %llu scan(s)\n", log_path.c_str(), db_path.c_str(),
                static_cast<unsigned long long>(total));

    const std::uint64_t limit = (max_scans != 0 && max_scans < total) ? max_scans : total;
    for (std::uint64_t i = 0; i < limit; ++i) {
      const lidar::Scan scan = reader.get(i);
      std::printf(
          "[%4llu] frame_id=%-12s stamp_ns=%-14lld points=%-4zu "
          "angle=[%.2f, %.2f] rad  range=[%.2f, %.2f] m",
          static_cast<unsigned long long>(i), scan.frame_id.c_str(),
          static_cast<long long>(scan.stamp_ns), scan.ranges.size(), static_cast<double>(scan.angle_min),
          static_cast<double>(scan.angle_max), static_cast<double>(scan.range_min),
          static_cast<double>(scan.range_max));
      if (!scan.ranges.empty()) {
        std::printf("  first=%.3f last=%.3f", static_cast<double>(scan.ranges.front()),
                    static_cast<double>(scan.ranges.back()));
      }
      std::printf("\n");
    }

    if (limit < total) {
      std::printf("... %llu more scan(s) not shown (pass a larger max_scans to see them)\n",
                  static_cast<unsigned long long>(total - limit));
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "dump_scans: %s\n", e.what());
    return 1;
  }

  return 0;
}