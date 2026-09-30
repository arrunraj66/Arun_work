#pragma once
//
// nav/pose_subscriber.hpp — PoseSubscriber, same shape as
// nav::NavSubscriber. This is what waypoint_follow_main.cpp links -- it
// never needs to know VehiclePose travels as protobuf over ZeroMQ
// underneath.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/vehicle_pose.hpp"

namespace nav {

/// Receives VehiclePoses from a PosePublisher.
class PoseSubscriber {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to.
  /// `topic` must match what the publisher sends under.
  explicit PoseSubscriber(std::string endpoint, std::string topic = "nav.pose");
  ~PoseSubscriber();

  PoseSubscriber(const PoseSubscriber&) = delete;
  PoseSubscriber& operator=(const PoseSubscriber&) = delete;
  PoseSubscriber(PoseSubscriber&&) noexcept;
  PoseSubscriber& operator=(PoseSubscriber&&) noexcept;

  /// True and `out` filled if a pose arrived inside the timeout; false on
  /// timeout OR on a message that failed to parse (counted in malformed()
  /// either way). Pass timeout_ms = 0 for a non-blocking poll -- the
  /// control loop pattern every other Subscriber in this project uses.
  [[nodiscard]] bool poll_pose(VehiclePose& out, int timeout_ms) noexcept;

  /// Messages received on our topic that could not be parsed as a
  /// VehiclePose. Should be 0.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
