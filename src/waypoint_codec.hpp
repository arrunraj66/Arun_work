#pragma once
//
// src/waypoint_codec.hpp -- the ONLY file that mentions both
// nav::WaypointMission/nav::VehiclePose and their proto counterparts. Same
// role nav_codec.hpp plays for nav::NavCommand: lives in src/, so no
// consumer of nav::nav ever sees protobuf.

#include "nav/vehicle_pose.hpp"
#include "nav/waypoint.hpp"
#include "nav_waypoint.pb.h"

namespace nav {

/// WaypointMission -> wire format. Overwrites everything already in `out`.
void to_proto(const WaypointMission& mission, proto::WaypointMission& out);

/// Wire format -> WaypointMission.
[[nodiscard]] WaypointMission from_proto(const proto::WaypointMission& msg);

/// VehiclePose -> wire format. Overwrites everything already in `out`.
void to_proto(const VehiclePose& pose, proto::VehiclePose& out);

/// Wire format -> VehiclePose.
[[nodiscard]] VehiclePose from_proto(const proto::VehiclePose& msg);

}  // namespace nav
