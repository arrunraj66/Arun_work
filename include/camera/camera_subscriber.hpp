#pragma once
//
// camera/camera_subscriber.hpp  CameraSubscriber, same shape as
// imu::ImuSubscriber / nav::NavSubscriber. This is what a C++ consumer
// (a recorder, a display, a future vision pipeline) links -- the Python
// Foxglove bridge instead decodes camera_frame_pb2 directly, since it
// already does the same for lidar and IMU.

#include <cstdint>
#include <memory>
#include <string>

#include "camera/frame.hpp"

namespace camera {

/// Receives camera Frames from a CameraPublisher.
class CameraSubscriber {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to.
  /// `topic` must match what the publisher sends under.
  explicit CameraSubscriber(std::string endpoint, std::string topic = "sensor.camera");
  ~CameraSubscriber();

  CameraSubscriber(const CameraSubscriber&) = delete;
  CameraSubscriber& operator=(const CameraSubscriber&) = delete;
  CameraSubscriber(CameraSubscriber&&) noexcept;
  CameraSubscriber& operator=(CameraSubscriber&&) noexcept;

  /// True and `out` filled if a frame arrived inside the timeout; false on
  /// timeout OR on a message that failed to parse (counted in malformed()
  /// either way).
  [[nodiscard]] bool poll_frame(Frame& out, int timeout_ms) noexcept;

  /// Messages received on our topic that could not be parsed as a Frame.
  /// Should be 0.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera