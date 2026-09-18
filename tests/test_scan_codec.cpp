// Round-trip: build a Scan by hand, serialise it to protobuf, deserialise it
// back, and assert every field survived unchanged. This is the test Step 2
// exists to make pass.

#include "lidar/scan.hpp"
#include "scan_codec.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  lidar::Scan original;
  original.frame_id          = "picoscan150_bow";
  original.stamp_ns           = 1'758'000'000'000'000'000LL;
  original.scan_time_ns       = 40'000'000;
  original.time_increment_ns  = 55'555;
  original.angle_min           = -2.409F;
  original.angle_max           =  2.409F;
  original.angle_increment     =  0.00349F;
  original.range_min           = 0.05F;
  original.range_max           = 120.0F;
  original.echo_index          = 0;
  original.echo_count          = 1;
  original.ranges              = {1.203F, 0.0F, 120.0F, 3.5F, 0.02F};
  original.intensities         = {};   // this sensor profile reports none

  lidar::proto::LidarScan wire;
  lidar::to_proto(original, wire);

  // Serialise and deserialise for real, not just object-to-object  the
  // whole point is proving the BYTES round-trip, not just the two functions.
  const std::string bytes = wire.SerializeAsString();
  lidar::proto::LidarScan wire2;
  check(wire2.ParseFromString(bytes), "the serialised bytes parse back as a LidarScan");

  const lidar::Scan restored = lidar::from_proto(wire2);

  check(restored.frame_id == original.frame_id, "frame_id survives");
  check(restored.stamp_ns == original.stamp_ns, "stamp_ns survives");
  check(restored.scan_time_ns == original.scan_time_ns, "scan_time_ns survives");
  check(restored.time_increment_ns == original.time_increment_ns, "time_increment_ns survives");
  check(restored.angle_min == original.angle_min, "angle_min survives exactly");
  check(restored.angle_max == original.angle_max, "angle_max survives exactly");
  check(restored.angle_increment == original.angle_increment, "angle_increment survives exactly");
  check(restored.range_min == original.range_min, "range_min survives exactly");
  check(restored.range_max == original.range_max, "range_max survives exactly");
  check(restored.echo_index == original.echo_index, "echo_index survives");
  check(restored.echo_count == original.echo_count, "echo_count survives");
  check(restored.ranges == original.ranges, "every range value survives, in order");
  check(restored.intensities.empty(), "an empty intensities vector stays empty, not null");

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}