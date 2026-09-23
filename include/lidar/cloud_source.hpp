#pragma once
#include "lidar/point_cloud.hpp"

namespace lidar {

class ICloudSource {
 public:
  virtual ~ICloudSource() = default;
  [[nodiscard]] virtual bool poll_cloud(PointCloud& out, int timeout_ms) noexcept = 0;
};

}  // namespace lidar