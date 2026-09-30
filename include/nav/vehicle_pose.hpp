#pragma once
//
// nav/vehicle_pose.hpp -- where the vehicle is right now. Same "plain
// struct" rule as nav::NavCommand and nav::Waypoint.

#include <cstdint>

namespace nav {

// World-frame metres, plus heading (yaw, radians, 0 = +x axis, positive =
// counter-clockwise looking down from above -- matches the Webots
// controller's existing yaw convention).
//
// Deliberately silent on WHERE this came from: today it is filled from the
// Webots Supervisor's ground-truth getPosition() (see the controller
// integration notes shipped with this module), but nothing downstream --
// WaypointFollower included -- has to change when it is filled from a real
// sensor-fusion estimate instead. Same "adapter fills the struct, the
// consumer doesn't know how" split every other module in this project
// follows (camera::Frame, imu::Imu, lidar::Scan).
struct VehiclePose {
  std::int64_t stamp_ns = 0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float heading_rad = 0.0F;
};

}  // namespace nav
