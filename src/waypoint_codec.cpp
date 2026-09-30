#include "waypoint_codec.hpp"

namespace nav {

void to_proto(const WaypointMission& mission, proto::WaypointMission& out) {
  out.Clear();
  out.set_mission_id(mission.mission_id);
  out.set_loop(mission.loop);

  for (const Waypoint& wp : mission.waypoints) {
    proto::Waypoint* wire_wp = out.add_waypoints();
    wire_wp->set_x(wp.x);
    wire_wp->set_y(wp.y);
    wire_wp->set_z(wp.z);
    wire_wp->set_arrival_radius(wp.arrival_radius);
  }
}

WaypointMission from_proto(const proto::WaypointMission& msg) {
  WaypointMission mission;
  mission.mission_id = msg.mission_id();
  mission.loop = msg.loop();

  mission.waypoints.reserve(static_cast<std::size_t>(msg.waypoints_size()));
  for (const proto::Waypoint& wire_wp : msg.waypoints()) {
    Waypoint wp;
    wp.x = wire_wp.x();
    wp.y = wire_wp.y();
    wp.z = wire_wp.z();
    wp.arrival_radius = wire_wp.arrival_radius();
    mission.waypoints.push_back(wp);
  }

  return mission;
}

void to_proto(const VehiclePose& pose, proto::VehiclePose& out) {
  out.set_stamp_ns(pose.stamp_ns);
  out.set_x(pose.x);
  out.set_y(pose.y);
  out.set_z(pose.z);
  out.set_heading_rad(pose.heading_rad);
}

VehiclePose from_proto(const proto::VehiclePose& msg) {
  VehiclePose pose;
  pose.stamp_ns = msg.stamp_ns();
  pose.x = msg.x();
  pose.y = msg.y();
  pose.z = msg.z();
  pose.heading_rad = msg.heading_rad();
  return pose;
}

}  // namespace nav
