// Round-trip: build a NavCommand by hand, serialise it to protobuf,
// deserialise it back, assert every field survived unchanged. Same shape
// as test_scan_codec.cpp / test_cloud_codec.cpp.

#include "nav/command.hpp"
#include "nav_codec.hpp"

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
  nav::NavCommand original;
  original.stamp_ns = 1'758'000'000'000'000'000LL;
  original.forward_speed = 0.42F;
  original.turn_rate = -0.73F;
  original.obstacle_detected = true;

  nav::proto::NavCommand wire;
  nav::to_proto(original, wire);

  check(wire.stamp_ns() == original.stamp_ns, "wire stamp_ns matches");
  check(wire.forward_speed() == original.forward_speed, "wire forward_speed matches");
  check(wire.turn_rate() == original.turn_rate, "wire turn_rate matches");
  check(wire.obstacle_detected() == original.obstacle_detected, "wire obstacle_detected matches");

  const std::string bytes = wire.SerializeAsString();
  nav::proto::NavCommand wire2;
  check(wire2.ParseFromString(bytes), "reparses from serialised bytes");

  const nav::NavCommand round_tripped = nav::from_proto(wire2);
  check(round_tripped.stamp_ns == original.stamp_ns, "round-trip stamp_ns matches");
  check(round_tripped.forward_speed == original.forward_speed, "round-trip forward_speed matches");
  check(round_tripped.turn_rate == original.turn_rate, "round-trip turn_rate matches");
  check(round_tripped.obstacle_detected == original.obstacle_detected,
        "round-trip obstacle_detected matches");

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
