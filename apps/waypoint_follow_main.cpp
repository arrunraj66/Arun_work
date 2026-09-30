// waypoint_follow_main.cpp -- Navigation Step 2's runnable half: subscribes
// to nav.pose (wherever the vehicle's current position estimate is
// published from -- the Webots controller's Supervisor ground truth today,
// a real sensor-fusion estimate later, see vehicle_pose.hpp) and
// nav.mission (a WaypointMission, sent by waypoint_mission_sender_main),
// decides a steering command with nav::follow(), and publishes it on
// nav.cmd -- the SAME topic nav_avoid_main.cpp uses. The Webots controller
// only ever needs one NavSubscriber on nav.cmd; which node is actually
// driving it (reactive avoidance vs waypoint following) is decided by
// which of these two processes you run, not by anything the controller
// has to know about.
//
// Mission bookkeeping (which waypoint index is current, when to advance,
// when to loop) lives HERE, not inside nav::follow() -- follow() is a pure
// per-tick decision, same separation nav_avoid_main.cpp keeps between
// itself and nav::decide().

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "nav/follower.hpp"
#include "nav/mission_subscriber.hpp"
#include "nav/nav_publisher.hpp"
#include "nav/pose_subscriber.hpp"

namespace {

[[nodiscard]] std::int64_t now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

int main(int argc, char** argv) {
  // Usage: waypoint_follow_main [pose_endpoint] [mission_endpoint] [nav_endpoint]
  //   pose_endpoint:    where the Webots controller's nav.pose publisher binds
  //   mission_endpoint: where waypoint_mission_sender_main binds
  //   nav_endpoint:     where THIS process binds its own nav.cmd publisher
  //                     (same default port nav_avoid_main.cpp uses -- run
  //                     one or the other, not both, against one controller)
  const std::string pose_endpoint = (argc > 1) ? argv[1] : "tcp://127.0.0.1:5585";
  const std::string mission_endpoint = (argc > 2) ? argv[2] : "tcp://127.0.0.1:5586";
  const std::string nav_endpoint = (argc > 3) ? argv[3] : "tcp://*:5657";

  std::cout << "waypoint_follow_main: subscribing to pose on " << pose_endpoint << std::endl;
  std::cout << "waypoint_follow_main: subscribing to mission on " << mission_endpoint
            << std::endl;
  std::cout << "waypoint_follow_main: publishing nav.cmd on " << nav_endpoint << std::endl;

  nav::PoseSubscriber pose_sub(pose_endpoint, "nav.pose");
  nav::MissionSubscriber mission_sub(mission_endpoint, "nav.mission");
  nav::NavPublisher publisher(nav_endpoint, "nav.cmd");

  const nav::FollowerConfig cfg{};  // defaults: see follower.hpp

  nav::VehiclePose pose{};
  bool have_pose = false;

  nav::WaypointMission mission{};
  // -1 so the very first mission received (mission_id defaults to 0 in a
  // freshly-constructed WaypointMission) is still recognised as new.
  std::int64_t known_mission_id = -1;
  std::size_t waypoint_index = 0;

  std::uint64_t ticks = 0;
  std::uint64_t published = 0;
  std::uint64_t dropped = 0;

  // Control loop rate: matches nav_avoid_main.cpp's 20 Hz, for the same
  // reason -- fast enough to react promptly, not so fast it busy-polls.
  constexpr auto kTickPeriod = std::chrono::milliseconds(50);
  constexpr int kPollTimeoutMs = 5;

  while (true) {
    nav::VehiclePose new_pose;
    if (pose_sub.poll_pose(new_pose, kPollTimeoutMs)) {
      pose = new_pose;
      have_pose = true;
    }

    nav::WaypointMission incoming;
    if (mission_sub.poll_mission(incoming, kPollTimeoutMs)) {
      if (incoming.mission_id != known_mission_id) {
        // A genuinely new mission -- start over from waypoint 0. A
        // re-broadcast of the SAME mission_id (the normal case, since the
        // sender keeps re-sending) falls through here and changes nothing.
        mission = incoming;
        known_mission_id = incoming.mission_id;
        waypoint_index = 0;
        std::cout << "waypoint_follow_main: new mission_id=" << known_mission_id << " with "
                  << mission.waypoints.size() << " waypoint(s)" << std::endl;
      }
    }

    nav::NavCommand cmd;
    cmd.stamp_ns = now_ns();

    const bool have_target = have_pose && !mission.waypoints.empty() &&
                              waypoint_index < mission.waypoints.size();

    if (have_target) {
      const nav::Waypoint& target = mission.waypoints[waypoint_index];
      const nav::FollowResult result = nav::follow(pose, target, cfg);
      cmd = result.command;
      cmd.stamp_ns = now_ns();

      if (result.arrived) {
        if (waypoint_index + 1 < mission.waypoints.size()) {
          ++waypoint_index;
          std::cout << "waypoint_follow_main: arrived at waypoint " << waypoint_index - 1
                     << ", advancing to " << waypoint_index << std::endl;
        } else if (mission.loop) {
          waypoint_index = 0;
          std::cout << "waypoint_follow_main: arrived at final waypoint, looping back to 0"
                     << std::endl;
        }
        // Else: stay on the last waypoint. follow() will keep reporting
        // arrived + a zero command every tick, which is the correct
        // "mission complete, hold station" behaviour.
      }
    }
    // Else: no pose yet, or no mission yet, or mission has zero waypoints
    // -- cmd stays default-constructed (forward_speed=0, turn_rate=0).
    // Same fail-safe rule as nav_avoid_main.cpp's SectorRanges default:
    // missing data must never look like "go ahead," only ever "hold."

    if (publisher.publish(cmd)) {
      ++published;
    } else {
      ++dropped;
    }

    ++ticks;
    if (ticks % 100 == 0) {
      std::cout << "waypoint_follow_main: tick " << ticks << " | have_pose=" << have_pose
                << " mission_id=" << known_mission_id << " waypoint=" << waypoint_index << "/"
                << mission.waypoints.size() << " | forward_speed=" << cmd.forward_speed
                << " turn_rate=" << cmd.turn_rate << " | published=" << published
                << " dropped=" << dropped << std::endl;
    }

    std::this_thread::sleep_for(kTickPeriod);
  }

  return 0;
}
