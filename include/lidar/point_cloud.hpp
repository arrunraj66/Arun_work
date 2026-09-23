#pragma once
//
// lidar/point_cloud.hpp -- the 3D counterpart of scan.hpp: one full frame
// from a multi-layer LiDAR (multiScan136), stored the way the sensor
// measured it -- range + two angles per point -- not as x/y/z.
//
// Same rule as scan.hpp: standard library only. No protobuf, no ZeroMQ,
// no vendor SDK. Anyone who wants 3D data compiles against this and
// nothing behind it.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lidar {

/// One full frame (every enabled layer, one revolution) of a 3D LiDAR.
///
/// Parallel arrays, not a vector of point structs: point i is
/// ranges[i] / azimuths[i] / elevations[i] / intensities[i]. This matches
/// how Scan stores data, and it maps 1:1 onto protobuf "packed repeated
/// float" fields -- 4 bytes per value on the wire, no per-point overhead.
struct PointCloud {
  std::string frame_id;             // which sensor produced this frame
  std::int64_t stamp_ns = 0;        // when the frame STARTED, ns since epoch
  std::int64_t scan_time_ns = 0;    // duration of one full frame (0 = unknown)

  float range_min = 0.0F;           // closest measurable distance, metres
  float range_max = 0.0F;           // furthest measurable distance, metres

  std::vector<float> ranges;        // metres, one per point
  std::vector<float> azimuths;      // radians, horizontal angle, one per point
  std::vector<float> elevations;    // radians, vertical angle (0 = level), one per point
  std::vector<float> intensities;   // return strength, one per point; may be empty

  [[nodiscard]] std::size_t size() const noexcept { return ranges.size(); }
};

/// A Cartesian point in the sensor's own frame, metres.
struct Point3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

/// Point i converted to x/y/z -- computed on demand, never stored, the same
/// way angle_of() is for Scan. x forward, y left, z up.
[[nodiscard]] inline Point3 to_xyz(const PointCloud& cloud, std::size_t i) noexcept {
  const float r = cloud.ranges[i];
  const float az = cloud.azimuths[i];
  const float el = cloud.elevations[i];
  const float horizontal = r * std::cos(el);  // length of the point's shadow on the floor
  return Point3{horizontal * std::cos(az), horizontal * std::sin(az), r * std::sin(el)};
}

/// Same convention as is_valid(Scan, i): a real measurement lies strictly
/// inside (range_min, range_max); "no return" does not count as a point.
[[nodiscard]] inline bool is_valid(const PointCloud& cloud, std::size_t i) noexcept {
  const float r = cloud.ranges[i];
  return r > cloud.range_min && r < cloud.range_max;
}

}  // namespace lidar