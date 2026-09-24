#pragma once
//
// lidar/cloud_source.hpp — the 3D counterpart of lidar/source.hpp. Same
// seam, same reasoning, one virtual method, one virtual destructor, no
// dependencies beyond point_cloud.hpp. Kept as a SEPARATE interface from
// IScanSource rather than templating one interface over both, because a
// consumer that only ever wants 2D Scans (the existing scan_subscriber_main,
// the recorder, the query server) should never be forced to know PointCloud
// exists -- same "public headers only mention what this caller actually
// needs" rule the rest of the library follows.

#include "lidar/point_cloud.hpp"

namespace lidar {

/// Anything that can produce PointClouds, polled rather than pushed.
class ICloudSource {
 public:
  virtual ~ICloudSource() = default;

  /// Waits up to timeout_ms for the next point cloud (one full frame from
  /// the sensor -- all active layers, one revolution).
  ///
  /// Returns true and fills `out` if a cloud arrived within the timeout;
  /// returns false on timeout. A timeout is the ordinary, expected outcome
  /// of asking "do you have anything yet" -- it is not an error, and an
  /// implementation must not throw for it. `out` is left unspecified on a
  /// false return; callers must not read it.
  [[nodiscard]] virtual bool poll_cloud(PointCloud& out, int timeout_ms) noexcept = 0;
};

}  // namespace lidar
