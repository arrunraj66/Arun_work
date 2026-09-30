// waypoint_mission_sender_main.cpp -- publishes a WaypointMission on
// nav.mission, repeatedly, forever (Ctrl+C to stop). Re-sending is
// deliberate, not a bug: a WaypointMission is not a per-tick stream, so a
// waypoint_follow_main started before or after this process would
// otherwise miss it entirely (ZeroMQ's "slow joiner" problem -- see
// mission_publisher.hpp). Re-sending the same mission_id every second
// means it doesn't matter which process you start first, and
// waypoint_follow_main already ignores a re-broadcast of a mission_id it
// has already started.
//
// To send a DIFFERENT route: edit the `waypoints` list below and bump
// kMissionId, then rebuild and rerun. Bumping the id is what tells a
// currently-running waypoint_follow_main "this is new, start over at
// waypoint 0" -- reusing the same id would just look like the same
// mission being re-sent.

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "nav/mission_publisher.hpp"

int main(int argc, char** argv) {
  // Usage: waypoint_mission_sender_main [mission_endpoint]
  const std::string mission_endpoint = (argc > 1) ? argv[1] : "tcp://*:5586";

  constexpr std::int64_t kMissionId = 1;

  // A simple square route, 10m per side, in world-frame metres -- matches
  // whatever frame the Webots Supervisor's getPosition() returns. Replace
  // with real coordinates for your scene.
  nav::WaypointMission mission;
  mission.mission_id = kMissionId;
  mission.loop = true;
  mission.waypoints = {
      {10.0F, 0.0F, 0.0F, 2.0F},
      {10.0F, 10.0F, 0.0F, 2.0F},
      {0.0F, 10.0F, 0.0F, 2.0F},
      {0.0F, 0.0F, 0.0F, 2.0F},
  };

  std::cout << "waypoint_mission_sender_main: publishing nav.mission on " << mission_endpoint
            << std::endl;
  std::cout << "waypoint_mission_sender_main: mission_id=" << mission.mission_id << " loop="
            << (mission.loop ? "true" : "false") << " waypoints=" << mission.waypoints.size()
            << std::endl;

  nav::MissionPublisher publisher(mission_endpoint, "nav.mission");

  std::uint64_t sent = 0;
  while (true) {
    if (publisher.publish(mission)) {
      ++sent;
    }

    if (sent % 10 == 0) {
      std::cout << "waypoint_mission_sender_main: sent " << sent << " time(s)" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
