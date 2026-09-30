#pragma once

#include <cstdint>

#include <memory>

#include <string>

#include "camera/frame.hpp"

namespace camera {

class CameraPublisher {
 public:
  explicit CameraPublisher(std::string endpoint, std::string topic = "sensor.camera");
  ~CameraPublisher();

  CameraPublisher(const CameraPublisher&) = delete;
  CameraPublisher& operator=(const CameraPublisher&) = delete;
  CameraPublisher(CameraPublisher&&) noexcept;
  CameraPublisher& operator=(CameraPublisher&&) noexcept;

  [[nodiscard]] bool publish(const Frame& frame) noexcept;
  [[nodiscard]] std::uint64_t dropped() const noexcept;
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera