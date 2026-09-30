// nav::follow() is a pure function, so this test is pure numbers in,
// numbers out -- no ZeroMQ, no protobuf, no Webots. Covers the heading
// sign convention (positive turn_rate = toward the left, matching
// NavCommand's own documented convention), the forward-speed taper as
// heading error grows, and the arrival check.

#include "nav/follower.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

constexpr float kPi = 3.14159265358979323846F;
constexpr nav::FollowerConfig kDefaultCfg{};  // cruise=1.0, max_turn=1.0, gain=1.0

}  // namespace

int main() {
  // Target straight ahead: no turning needed, full cruise speed, not
  // arrived (well outside arrival_radius).
  {
    const nav::VehiclePose pose{/*stamp_ns=*/0, /*x=*/0.0F, /*y=*/0.0F, /*z=*/0.0F,
                                 /*heading_rad=*/0.0F};
    const nav::Waypoint target{/*x=*/10.0F, /*y=*/0.0F, /*z=*/0.0F, /*arrival_radius=*/1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(!result.arrived, "ahead: not yet arrived");
    check(std::fabs(result.command.turn_rate) < 1.0e-4F, "ahead: no turning needed");
    check(std::fabs(result.command.forward_speed - kDefaultCfg.cruise_speed) < 1.0e-4F,
          "ahead: full cruise speed");
    check(std::fabs(result.distance_remaining - 10.0F) < 1.0e-4F,
          "ahead: distance_remaining is 10");
  }

  // Target to the left (positive Y, vehicle facing +X): turn_rate must be
  // positive, matching NavCommand's documented "positive = turn left".
  {
    const nav::VehiclePose pose{0, 0.0F, 0.0F, 0.0F, 0.0F};
    const nav::Waypoint target{0.0F, 10.0F, 0.0F, 1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(result.command.turn_rate > 0.0F, "left target: turn_rate is positive (turn left)");
    check(result.command.forward_speed < kDefaultCfg.cruise_speed,
          "left target: forward_speed tapered down while turning");
  }

  // Target to the right (negative Y): turn_rate must be negative.
  {
    const nav::VehiclePose pose{0, 0.0F, 0.0F, 0.0F, 0.0F};
    const nav::Waypoint target{0.0F, -10.0F, 0.0F, 1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(result.command.turn_rate < 0.0F, "right target: turn_rate is negative (turn right)");
  }

  // Target directly behind: a 180-degree boundary case (same spirit as
  // AvoiderConfig's tie-break -- the exact sign is arbitrary but the
  // magnitude must saturate to max_turn_rate, and forward_speed must be
  // zero, never negative/reversing).
  {
    const nav::VehiclePose pose{0, 0.0F, 0.0F, 0.0F, 0.0F};
    const nav::Waypoint target{-10.0F, 0.0F, 0.0F, 1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(std::fabs(std::fabs(result.command.turn_rate) - kDefaultCfg.max_turn_rate) < 1.0e-3F,
          "behind: turn_rate saturates to max_turn_rate");
    check(result.command.forward_speed >= 0.0F && result.command.forward_speed < 1.0e-3F,
          "behind: forward_speed is zero, never negative");
  }

  // max_turn_rate must never be exceeded, however large the heading error.
  {
    const nav::VehiclePose pose{0, 0.0F, 0.0F, 0.0F, kPi};  // facing -X
    const nav::Waypoint target{0.0F, 10.0F, 0.0F, 1.0F};    // target is +Y
    nav::FollowerConfig cfg = kDefaultCfg;
    cfg.turn_gain = 5.0F;  // deliberately large gain to try to overshoot the clamp
    const nav::FollowResult result = nav::follow(pose, target, cfg);

    check(std::fabs(result.command.turn_rate) <= cfg.max_turn_rate + 1.0e-4F,
          "turn_rate never exceeds max_turn_rate even with a large gain");
  }

  // Arrival: inside arrival_radius reports arrived and a full stop, exactly
  // at the boundary counts as arrived (<=, not <).
  {
    const nav::VehiclePose pose{0, 9.5F, 0.0F, 0.0F, 0.0F};
    const nav::Waypoint target{10.0F, 0.0F, 0.0F, /*arrival_radius=*/1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(result.arrived, "inside arrival_radius: arrived is true");
    check(result.command.forward_speed == 0.0F, "arrived: forward_speed is zero");
    check(result.command.turn_rate == 0.0F, "arrived: turn_rate is zero");
  }
  {
    const nav::VehiclePose pose{0, 0.0F, 0.0F, 0.0F, 0.0F};
    const nav::Waypoint target{1.0F, 0.0F, 0.0F, /*arrival_radius=*/1.0F};
    const nav::FollowResult result = nav::follow(pose, target, kDefaultCfg);

    check(result.arrived, "exactly at arrival_radius boundary: arrived is true (<=, not <)");
  }

  if (failures > 0) {
    std::printf("%d check(s) FAILED\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all checks passed\n");
  return EXIT_SUCCESS;
}
