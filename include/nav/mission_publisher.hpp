#pragma once
//
// nav/mission_publisher.hpp — MissionPublisher, same shape as
// nav::NavPublisher. Built on mw::Publisher; hides that a WaypointMission
// has to become bytes before it can leave this process.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/waypoint.hpp"

namespace nav {

/// Publishes WaypointMissions to anyone who subscribes -- in practice, a
/// waypoint_follow_main process. Same fan-out, fire-and-forget contract as
/// every other Publisher in this project.
///
/// A WaypointMission is not a per-tick stream like NavCommand -- it only
/// changes when someone sets a new mission. That means a subscriber that
/// connects late (the ZeroMQ "slow joiner" problem -- see
/// MissionSubscriber's own comment) can permanently miss it if it is only
/// sent once. The fix is on the SENDER side, not this class: whatever
/// calls publish() should call it repeatedly (e.g. once a second) for as
/// long as that mission should be in effect, not just a single time. See
/// waypoint_mission_sender_main.cpp for the pattern.
class MissionPublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS.
  /// `topic` defaults to "nav.mission" -- distinct from "nav.cmd" and
  /// "nav.pose" so nothing downstream can mix them up.
  explicit MissionPublisher(std::string endpoint, std::string topic = "nav.mission");
  ~MissionPublisher();

  MissionPublisher(const MissionPublisher&) = delete;
  MissionPublisher& operator=(const MissionPublisher&) = delete;
  MissionPublisher(MissionPublisher&&) noexcept;
  MissionPublisher& operator=(MissionPublisher&&) noexcept;

  /// Serialises `mission` and sends it. Returns false if the send queue
  /// was full and the mission was dropped.
  [[nodiscard]] bool publish(const WaypointMission& mission) noexcept;

  /// How many missions this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
