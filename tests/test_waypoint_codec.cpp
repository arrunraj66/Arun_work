// Round-trip: build a WaypointMission and a VehiclePose by hand, serialise,
// deserialise, assert every field survived unchanged. Same shape as
// test_nav_codec.cpp / test_camera_codec.cpp.

#include "nav/vehicle_pose.hpp"
#include "nav/waypoint.hpp"
#include "waypoint_codec.hpp"

#include <cstdio>
#include <cstdlib>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  // --- WaypointMission round-trip ---
  {
    nav::WaypointMission original;
    original.mission_id = 42;
    original.loop = true;
    original.waypoints.push_back({10.0F, 0.0F, -1.5F, 2.0F});
    original.waypoints.push_back({10.0F, 10.0F, -1.5F, 2.0F});
    original.waypoints.push_back({0.0F, 10.0F, -1.5F, 2.0F});

    nav::proto::WaypointMission wire;
    nav::to_proto(original, wire);

    check(wire.mission_id() == original.mission_id, "wire mission_id matches");
    check(wire.loop() == original.loop, "wire loop matches");
    check(wire.waypoints_size() == 3, "wire has 3 waypoints");

    const std::string bytes = wire.SerializeAsString();
    nav::proto::WaypointMission wire2;
    check(wire2.ParseFromString(bytes), "reparses from serialised bytes");

    const nav::WaypointMission round_tripped = nav::from_proto(wire2);
    check(round_tripped.mission_id == original.mission_id, "round-trip mission_id matches");
    check(round_tripped.loop == original.loop, "round-trip loop matches");
    check(round_tripped.waypoints.size() == original.waypoints.size(),
          "round-trip waypoint count matches");

    bool all_waypoints_match = round_tripped.waypoints.size() == original.waypoints.size();
    if (all_waypoints_match) {
      for (std::size_t i = 0; i < original.waypoints.size(); ++i) {
        const nav::Waypoint& a = original.waypoints[i];
        const nav::Waypoint& b = round_tripped.waypoints[i];
        if (a.x != b.x || a.y != b.y || a.z != b.z || a.arrival_radius != b.arrival_radius) {
          all_waypoints_match = false;
          break;
        }
      }
    }
    check(all_waypoints_match, "round-trip waypoints match field-for-field, in order");
  }

  // --- Empty mission (zero waypoints) round-trip -- must not crash or
  // silently invent a waypoint. ---
  {
    nav::WaypointMission original;
    original.mission_id = 7;
    original.loop = false;

    nav::proto::WaypointMission wire;
    nav::to_proto(original, wire);
    const nav::WaypointMission round_tripped = nav::from_proto(wire);

    check(round_tripped.waypoints.empty(), "empty mission round-trips with zero waypoints");
  }

  // --- VehiclePose round-trip ---
  {
    nav::VehiclePose original;
    original.stamp_ns = 1'758'900'000'000'000'000LL;
    original.x = 3.25F;
    original.y = -7.5F;
    original.z = -1.0F;
    original.heading_rad = 1.5707963F;

    nav::proto::VehiclePose wire;
    nav::to_proto(original, wire);

    check(wire.stamp_ns() == original.stamp_ns, "wire stamp_ns matches");
    check(wire.x() == original.x, "wire x matches");
    check(wire.y() == original.y, "wire y matches");
    check(wire.z() == original.z, "wire z matches");
    check(wire.heading_rad() == original.heading_rad, "wire heading_rad matches");

    const std::string bytes = wire.SerializeAsString();
    nav::proto::VehiclePose wire2;
    check(wire2.ParseFromString(bytes), "reparses from serialised bytes");

    const nav::VehiclePose round_tripped = nav::from_proto(wire2);
    check(round_tripped.stamp_ns == original.stamp_ns, "round-trip stamp_ns matches");
    check(round_tripped.x == original.x, "round-trip x matches");
    check(round_tripped.y == original.y, "round-trip y matches");
    check(round_tripped.z == original.z, "round-trip z matches");
    check(round_tripped.heading_rad == original.heading_rad, "round-trip heading_rad matches");
  }

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
