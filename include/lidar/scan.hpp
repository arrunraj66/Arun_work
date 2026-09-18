
#pragma once
//
// lidar/scan.hpp — the one type every consumer of this library actually
// wants: a 2D LiDAR scan, stored the way the sensor measured it.
//
// No protobuf here, no ZeroMQ, no SQLite. This header has exactly three
// standard-library includes and nothing else may ever be added to it — that
// is the whole point of it being the PUBLIC type. The GUI team compiles
// against this file and nothing behind it.

#include <cstdint>

#include <string>

#include <vector>

namespace lidar {

/// One full revolution's worth of range measurements, plus everything needed
/// to interpret them. Angles are implied by array index, not stored per
/// sample — see angle_of() below.
struct Scan {
  std::string frame_id;             // which sensor produced this scan
  std::int64_t stamp_ns = 0;        // when the scan STARTED, ns since epoch
  std::int64_t scan_time_ns = 0;    // duration of one full revolution
  std::int64_t time_increment_ns = 0;  // time between consecutive samples

  float angle_min = 0.0F;           // angle of the first sample, radians
  float angle_max = 0.0F;           // angle of the last sample, radians
  float angle_increment = 0.0F;     // angular step between samples, radians

  float range_min = 0.0F;           // closest measurable distance, metres
  float range_max = 0.0F;           // furthest measurable distance, metres

  // The picoScan150 evaluates three echoes per angular step. Both fields
  // default to "this is an ordinary single-echo scan" so nothing that reads
  // a Scan today has to change when a multi-echo source appears later.
  std::uint8_t echo_index = 0;      // which echo this scan represents: 0 = first/nearest
  std::uint8_t echo_count = 1;      // how many echoes the sensor was configured to report

  std::vector<float> ranges;        // one distance per angular step, metres
  std::vector<float> intensities;   // one return strength per step; may be empty
};

/// The angle, in radians, of sample i — computed, never stored, because it
/// is fully determined by angle_min and angle_increment.
[[nodiscard]] inline float angle_of(const Scan& scan, std::size_t i) noexcept {
  return scan.angle_min + static_cast<float>(i) * scan.angle_increment;
}

/// Whether ranges[i] is an actual measurement rather than "no return". The
/// convention: valid means strictly inside (range_min, range_max) — a sensor
/// with nothing to see does not get to silently masquerade as a real 0.05 m.
[[nodiscard]] inline bool is_valid(const Scan& scan, std::size_t i) noexcept {
  const float r = scan.ranges[i];
  return r > scan.range_min && r < scan.range_max;
}

}  // namespace lidar