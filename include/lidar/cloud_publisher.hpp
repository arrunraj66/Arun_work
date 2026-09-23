#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "lidar/point_cloud.hpp"

namespace lidar {

class CloudPublisher {
 public:
  explicit CloudPublisher(std::string endpoint, std::string topic = "lidar.cloud");
  ~CloudPublisher();

  CloudPublisher(const CloudPublisher&) = delete;
  CloudPublisher& operator=(const CloudPublisher&) = delete;
  CloudPublisher(CloudPublisher&&) noexcept;
  CloudPublisher& operator=(CloudPublisher&&) noexcept;

  [[nodiscard]] bool publish(const PointCloud& cloud) noexcept;
  [[nodiscard]] std::uint64_t dropped() const noexcept;
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar