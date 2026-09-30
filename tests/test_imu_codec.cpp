// Round-trip: build an Imu reading by hand, serialise it to protobuf,
// deserialise it back, assert every field survived unchanged. Same shape as
// test_nav_codec.cpp / test_scan_codec.cpp / test_cloud_codec.cpp.

#include "imu/imu.hpp"
#include "imu_codec.hpp"

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
  // Case 1: a reading WITH a fused orientation estimate.
  {
    imu::Imu original;
    original.frame_id = "imu_link";
    original.stamp_ns = 1'758'000'000'000'000'000LL;
    original.has_orientation = true;
    original.orientation = {0.0, 0.0, 0.7071, 0.7071};
    original.angular_velocity = {0.01, -0.02, 0.03};
    original.linear_acceleration = {0.011971, -0.566246, 9.731529};  // real SICK sample

    imu::proto::Imu wire;
    imu::to_proto(original, wire);

    check(wire.frame_id() == original.frame_id, "wire frame_id matches");
    check(wire.stamp_ns() == original.stamp_ns, "wire stamp_ns matches");
    check(wire.orientation_valid() == original.has_orientation, "wire orientation_valid matches");
    check(wire.orientation().w() == original.orientation.w, "wire orientation.w matches");
    check(wire.linear_acceleration().z() == original.linear_acceleration.z,
          "wire linear_acceleration.z matches (gravity component)");

    const std::string bytes = wire.SerializeAsString();
    imu::proto::Imu wire2;
    check(wire2.ParseFromString(bytes), "reparses from serialised bytes");

    const imu::Imu round_tripped = imu::from_proto(wire2);
    check(round_tripped.frame_id == original.frame_id, "round-trip frame_id matches");
    check(round_tripped.stamp_ns == original.stamp_ns, "round-trip stamp_ns matches");
    check(round_tripped.has_orientation == original.has_orientation,
          "round-trip has_orientation matches");
    check(round_tripped.orientation.x == original.orientation.x, "round-trip orientation.x matches");
    check(round_tripped.orientation.y == original.orientation.y, "round-trip orientation.y matches");
    check(round_tripped.orientation.z == original.orientation.z, "round-trip orientation.z matches");
    check(round_tripped.orientation.w == original.orientation.w, "round-trip orientation.w matches");
    check(round_tripped.angular_velocity.x == original.angular_velocity.x,
          "round-trip angular_velocity.x matches");
    check(round_tripped.angular_velocity.y == original.angular_velocity.y,
          "round-trip angular_velocity.y matches");
    check(round_tripped.angular_velocity.z == original.angular_velocity.z,
          "round-trip angular_velocity.z matches");
    check(round_tripped.linear_acceleration.x == original.linear_acceleration.x,
          "round-trip linear_acceleration.x matches");
    check(round_tripped.linear_acceleration.y == original.linear_acceleration.y,
          "round-trip linear_acceleration.y matches");
    check(round_tripped.linear_acceleration.z == original.linear_acceleration.z,
          "round-trip linear_acceleration.z matches");
  }

  // Case 2: a reading with NO orientation (has_orientation = false) -- the
  // default-constructed Quaternion (identity, w=1) must still round-trip
  // correctly even though no consumer is supposed to trust it.
  {
    imu::Imu original;
    original.frame_id = "imu_link";
    original.stamp_ns = 42;
    original.has_orientation = false;
    original.angular_velocity = {0.0, 0.0, 0.0};
    original.linear_acceleration = {0.0, 0.0, 9.81};

    imu::proto::Imu wire;
    imu::to_proto(original, wire);
    const imu::Imu round_tripped = imu::from_proto(wire);

    check(!round_tripped.has_orientation, "has_orientation=false round-trips as false");
    check(round_tripped.orientation.w == 1.0, "default orientation quaternion still round-trips");
  }

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
