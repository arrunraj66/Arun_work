#pragma once
#include "lidar/scan.hpp"

namespace lidar {

class IScanSource {
 public:
  virtual ~IScanSource() = default;

  [[nodiscard]] virtual bool poll_scan(Scan& out, int timeout_ms) noexcept = 0;
};

}  // namespace lidar