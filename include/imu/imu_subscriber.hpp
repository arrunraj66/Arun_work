#pragma once
//
// imu/imu_subscriber.hpp — ImuSubscriber, same shape as nav::NavSubscriber /
// lidar::CloudSubscriber. This is what a fusion filter (or any other
// consumer) links -- it never needs to know Imu travels as protobuf over
// ZeroMQ underneath.

#include <cstdint>
#include <memory>
#include <string>

#include "imu/imu.hpp"

namespace imu {

/// Receives Imu readings from an ImuPublisher.
class ImuSubscriber {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to.
  /// `topic` must match what the publisher sends under.
  ///
  /// Same slow-joiner caveat as every other Subscriber in this project: a
  /// subscriber started after the publisher misses whatever was sent before
  /// it finished connecting.
  explicit ImuSubscriber(std::string endpoint, std::string topic = "sensor.imu");
  ~ImuSubscriber();

  ImuSubscriber(const ImuSubscriber&) = delete;
  ImuSubscriber& operator=(const ImuSubscriber&) = delete;
  ImuSubscriber(ImuSubscriber&&) noexcept;
  ImuSubscriber& operator=(ImuSubscriber&&) noexcept;

  /// True and `out` filled if a reading arrived inside the timeout; false
  /// on timeout OR on a message that failed to parse (counted in
  /// malformed() either way) -- same "false just means nothing usable
  /// arrived this call" contract as IImuSource::poll_imu.
  [[nodiscard]] bool poll_imu(Imu& out, int timeout_ms) noexcept;

  /// Messages received on our topic that could not be parsed as an Imu
  /// reading. Should be 0.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace imu
