// Round-trip for the 3D type: build a PointCloud by hand, serialise it to
// real protobuf bytes, parse them back, and check every field survived.
// Same shape as test_scan_codec.cpp.

#include "lidar/point_cloud.hpp"
#include "cloud_codec.hpp"

#include <cstdio>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  lidar::PointCloud original;
  original.frame_id     = "multiscan136";
  original.stamp_ns     = 1'790'153'097'553'984'165LL;  // a real stamp from the sensor log
  original.scan_time_ns = 50'000'000;                    // 20 Hz
  original.range_min    = 0.05F;
  original.range_max    = 120.0F;
  original.ranges       = {0.844F, 3.554F, 0.0F, 12.5F};
  original.azimuths     = {-3.068F, 0.0F, 1.57F, 3.14F};
  original.elevations   = {-0.747F, 0.0F, 0.131F, -0.2F};
  original.intensities  = {100.0F, 250.0F, 0.0F, 33.0F};

  lidar::proto::LidarPointCloud wire;
  lidar::to_proto(original, wire);
  const std::string bytes = wire.SerializeAsString();

  lidar::proto::LidarPointCloud wire2;
  check(wire2.ParseFromString(bytes), "the serialised bytes parse back as a LidarPointCloud");

  const lidar::PointCloud restored = lidar::from_proto(wire2);
  check(restored.frame_id == original.frame_id, "frame_id survives");
  check(restored.stamp_ns == original.stamp_ns, "stamp_ns survives");
  check(restored.scan_time_ns == original.scan_time_ns, "scan_time_ns survives");
  check(restored.range_min == original.range_min, "range_min survives exactly");
  check(restored.range_max == original.range_max, "range_max survives exactly");
  check(restored.ranges == original.ranges, "every range survives, in order");
  check(restored.azimuths == original.azimuths, "every azimuth survives, in order");
  check(restored.elevations == original.elevations, "every elevation survives, in order");
  check(restored.intensities == original.intensities, "every intensity survives, in order");
  check(restored.size() == 4, "size() reports 4 points");

  // Reusing `wire` for a smaller cloud must not leave old points behind --
  // that is exactly what a publisher does, frame after frame.
  lidar::PointCloud small = original;
  small.ranges = {1.0F};
  small.azimuths = {0.0F};
  small.elevations = {0.0F};
  small.intensities = {};
  lidar::to_proto(small, wire);
  check(wire.ranges_size() == 1 && wire.intensities_size() == 0,
        "reusing the message clears the previous frame's points");

  // A real frame is ~9,240 points (measured on the multiScan136). Check the
  // on-the-wire size is what the packed encoding promises: ~16 bytes/point.
  lidar::PointCloud big;
  big.frame_id = "multiscan136";
  big.ranges.assign(9240, 3.5F);
  big.azimuths.assign(9240, 1.0F);
  big.elevations.assign(9240, -0.3F);
  big.intensities.assign(9240, 200.0F);
  lidar::to_proto(big, wire);
  const std::size_t big_bytes = wire.SerializeAsString().size();
  std::printf("  a 9240-point frame serialises to %zu bytes (%.1f bytes/point)\n", big_bytes,
              static_cast<double>(big_bytes) / 9240.0);
  check(big_bytes < 9240 * 17, "packed encoding: under 17 bytes per point");

  std::printf("\n%s\n", failures == 0 ? "ALL PASSED" : "FAILURES");
  return failures == 0 ? 0 : 1;
}