// Step 1 proof: fill in a Scan by hand, print three of its angles, and check
// is_valid() against a couple of hand-picked ranges. Nothing here touches a
// sensor, a network, or a file — that is deliberate. This only has to prove
// the data type is right.

#include "lidar/scan.hpp"

#include <cstdio>

int main() {
  lidar::Scan scan;
  scan.frame_id          = "picoscan150_bow";
  scan.stamp_ns           = 1'758'000'000'000'000'000LL;
  scan.scan_time_ns       = 40'000'000;      // 40 ms -> 25 Hz
  scan.time_increment_ns  = 55'555;          // ~721 samples per revolution
  scan.angle_min           = -2.409F;         // -138 degrees, in radians
  scan.angle_max           =  2.409F;         // +138 degrees
  scan.angle_increment     =  0.00349F;       // ~0.2 degrees per step
  scan.range_min           = 0.05F;
  scan.range_max           = 120.0F;

  // A handful of samples: one clean return, one "too close to trust", one
  // "no return at all" (reported at exactly range_max), one ordinary return.
  scan.ranges = {1.203F, 0.0F, 120.0F, 3.5F, 0.02F};

  std::printf("frame_id = %s\n", scan.frame_id.c_str());
  std::printf("echo %u of %u\n", scan.echo_index, scan.echo_count);
  std::printf("\n%-6s %-12s %-10s %s\n", "i", "angle(rad)", "range(m)", "valid?");
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    std::printf("%-6zu %-12.5f %-10.3f %s\n",
                i, static_cast<double>(lidar::angle_of(scan, i)),
                static_cast<double>(scan.ranges[i]),
                lidar::is_valid(scan, i) ? "yes" : "no");
  }
}