#pragma once
//
// nav/waypoint.hpp -- the plain types a waypoint mission is made of, same
// "no protobuf, no ZeroMQ" rule as nav::NavCommand and every other struct
// in this project.

#include <cstdint>
#include <vector>

namespace nav {

struct Waypoint {
  // World-frame metres -- same frame the Webots Supervisor's own
  // getPosition() returns, so no conversion is needed to compare the two.
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;

  // Counts as "arrived" once within this distance (metres) of the point.
  float arrival_radius = 1.0F;
};

struct WaypointMission {
  // Bumped by whoever creates a mission every time the waypoint LIST
  // changes. A follower must ONLY reset its progress (back to waypoint 0)
  // when this differs from the mission_id it already has -- a mission
  // re-broadcast unchanged (which happens routinely; see
  // mission_publisher.hpp) must never look like a brand new mission and
  // restart progress from the beginning.
  std::int64_t mission_id = 0;

  // If true, after arriving at the last waypoint the follower should
  // return to waypoints[0] and continue, rather than stopping.
  bool loop = false;

  std::vector<Waypoint> waypoints;
};

}  // namespace nav
