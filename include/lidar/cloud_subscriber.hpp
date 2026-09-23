#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "lidar/cloud_source.hpp"

namespace lidar {

class CloudSubscriber final : public ICloudSource {
 public:
  explicit CloudSubscriber(std::string endpoint, std::string topic = "lidar.cloud");
  ~CloudSubscriber() override;

  CloudSubscriber(const CloudSubscriber&) = delete;
  CloudSubscriber& operator=(const CloudSubscriber&) = delete;
  CloudSubscriber(CloudSubscriber&&) noexcept;
  CloudSubscriber& operator=(CloudSubscriber&&) noexcept;

  [[nodiscard]] bool poll_cloud(PointCloud& out, int timeout_ms) noexcept override;
  [[nodiscard]] std::uint64_t malformed() const noexcept;
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar