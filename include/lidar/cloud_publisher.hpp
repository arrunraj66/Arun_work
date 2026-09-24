#pragma once
//
// lidar/cloud_publisher.hpp — CloudPublisher, the 3D counterpart of
// ScanPublisher. Same shape, same reasoning: built on mw::Publisher, hides
// the fact that a PointCloud has to become bytes before it can leave this
// process. Neither <zmq.hpp>, the protobuf headers, nor mw/transport.hpp
// appear here -- a consumer of lidar::lidar links none of them.

#include <cstdint>
#include <memory>
#include <string>

#include "lidar/point_cloud.hpp"

namespace lidar {

/// Publishes PointClouds to anyone who subscribes. Same fan-out,
/// fire-and-forget contract as ScanPublisher: never blocks, a cloud sent
/// with nobody listening is simply gone.
class CloudPublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS -- see
  /// ScanPublisher's constructor for the tcp:// vs ipc:// tradeoff.
  ///
  /// `topic` is deliberately a DIFFERENT default from ScanPublisher's
  /// ("lidar.cloud" vs "lidar.scan") so the two can share one process, one
  /// mw::Context, even one bound port's worth of subscribers without a
  /// CloudSubscriber ever accidentally receiving a 2D Scan's bytes and
  /// failing to parse them.
  explicit CloudPublisher(std::string endpoint, std::string topic = "lidar.cloud");
  ~CloudPublisher();

  CloudPublisher(const CloudPublisher&) = delete;
  CloudPublisher& operator=(const CloudPublisher&) = delete;
  CloudPublisher(CloudPublisher&&) noexcept;
  CloudPublisher& operator=(CloudPublisher&&) noexcept;

  /// Serialises `cloud` and sends it. Returns false if the send queue was
  /// full and the cloud was dropped -- same "not an error, check dropped()"
  /// contract as ScanPublisher::publish.
  [[nodiscard]] bool publish(const PointCloud& cloud) noexcept;

  /// How many clouds this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar
