#pragma once
//
// nav/mission_subscriber.hpp — MissionSubscriber, same shape as
// nav::NavSubscriber. This is what waypoint_follow_main.cpp links -- it
// never needs to know WaypointMission travels as protobuf over ZeroMQ
// underneath.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/waypoint.hpp"

namespace nav {

/// Receives WaypointMissions from a MissionPublisher.
class MissionSubscriber {
 public:
  /// `endpoint` is a ZeroMQ address, which this subscriber CONNECTS to.
  /// `topic` must match what the publisher sends under.
  ///
  /// Same slow-joiner caveat as every other Subscriber in this project: a
  /// MissionSubscriber started after the publisher's FIRST send of a given
  /// mission will simply see the next one -- which is why
  /// MissionPublisher's sender is expected to keep re-sending the current
  /// mission periodically (see mission_publisher.hpp).
  explicit MissionSubscriber(std::string endpoint, std::string topic = "nav.mission");
  ~MissionSubscriber();

  MissionSubscriber(const MissionSubscriber&) = delete;
  MissionSubscriber& operator=(const MissionSubscriber&) = delete;
  MissionSubscriber(MissionSubscriber&&) noexcept;
  MissionSubscriber& operator=(MissionSubscriber&&) noexcept;

  /// True and `out` filled if a mission arrived inside the timeout; false
  /// on timeout OR on a message that failed to parse (counted in
  /// malformed() either way). Pass timeout_ms = 0 for a non-blocking poll.
  ///
  /// Note this returns true on EVERY mission received, including a
  /// re-broadcast of the same mission_id -- it is the caller's job (see
  /// waypoint_follow_main.cpp) to compare mission_id against whatever it
  /// is already following and only reset progress when it actually
  /// changed.
  [[nodiscard]] bool poll_mission(WaypointMission& out, int timeout_ms) noexcept;

  /// Messages received on our topic that could not be parsed as a
  /// WaypointMission. Should be 0.
  [[nodiscard]] std::uint64_t malformed() const noexcept;

  /// The topic this subscriber is filtering on.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
