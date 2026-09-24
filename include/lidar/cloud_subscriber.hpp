#pragma once
//
// lidar/cloud_subscriber.hpp — CloudSubscriber, the 3D counterpart of
// ScanSubscriber. Same reasoning: an ICloudSource, so anything already
// written against that interface -- a future recorder, a GUI, an obstacle
// detector -- works unchanged whether the cloud came straight off
// SickCloudSource in this process or over the network from another one.

#include <cstdint>
#include <memory>
#include <string>

#include "lidar/cloud_source.hpp"

namespace lidar {

/// Receives PointClouds from a CloudPublisher.
class CloudSubscriber final : public ICloudSource {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to --
  /// the mirror of the publisher's bind. `topic` must match what the
  /// publisher sends under; ZeroMQ filters on it so clouds for other
  /// topics never reach this process at all.
  ///
  /// Same slow-joiner caveat as ScanSubscriber: a subscriber started after
  /// the publisher misses whatever was sent before it finished connecting.
  /// At ~20 Hz that is a non-issue for anything that just wants the latest
  /// cloud.
  explicit CloudSubscriber(std::string endpoint, std::string topic = "lidar.cloud");
  ~CloudSubscriber() override;

  CloudSubscriber(const CloudSubscriber&) = delete;
  CloudSubscriber& operator=(const CloudSubscriber&) = delete;
  CloudSubscriber(CloudSubscriber&&) noexcept;
  CloudSubscriber& operator=(CloudSubscriber&&) noexcept;

  /// ICloudSource's contract, unchanged: true and `out` filled if a cloud
  /// arrived inside the timeout, false on timeout. A message that arrives
  /// but fails to parse is counted in malformed() and treated as "nothing
  /// arrived".
  [[nodiscard]] bool poll_cloud(PointCloud& out, int timeout_ms) noexcept override;

  /// Messages received on our topic that could not be parsed as a
  /// PointCloud. Should be 0; anything else means a version mismatch
  /// between the publisher's wire format and ours.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar
