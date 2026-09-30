#pragma once
//
// imu/imu_publisher.hpp — ImuPublisher, same shape as nav::NavPublisher /
// lidar::CloudPublisher. Built on mw::Publisher; hides that an Imu reading
// has to become bytes before it can leave this process. Neither <zmq.hpp>,
// the protobuf headers, nor mw/transport.hpp appear here.

#include <cstdint>
#include <memory>
#include <string>

#include "imu/imu.hpp"

namespace imu {

/// Publishes Imu readings to anyone who subscribes. Same fan-out,
/// fire-and-forget contract as every other Publisher in this project: never
/// blocks, a reading sent with nobody listening is simply gone.
class ImuPublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS.
  /// `topic` defaults to "sensor.imu" -- distinct from every lidar/nav
  /// topic so nothing downstream can mix them up.
  explicit ImuPublisher(std::string endpoint, std::string topic = "sensor.imu");
  ~ImuPublisher();

  ImuPublisher(const ImuPublisher&) = delete;
  ImuPublisher& operator=(const ImuPublisher&) = delete;
  ImuPublisher(ImuPublisher&&) noexcept;
  ImuPublisher& operator=(ImuPublisher&&) noexcept;

  /// Serialises `msg` and sends it. Returns false if the send queue was
  /// full and the reading was dropped.
  [[nodiscard]] bool publish(const Imu& msg) noexcept;

  /// How many readings this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace imu
