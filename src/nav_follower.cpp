#include "nav/follower.hpp"

#include <algorithm>
#include <cmath>

namespace nav {

namespace {

constexpr float kPi = 3.14159265358979323846F;

// Wrap any angle (radians) into (-pi, pi], so a heading error of e.g.
// +350 degrees reads as -10 degrees -- "turn a little right," never "turn
// almost all the way around the long way."
[[nodiscard]] float wrap_pi(float angle) noexcept {
  constexpr float kTwoPi = 2.0F * kPi;
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0F) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

}  // namespace

FollowResult follow(const VehiclePose& current, const Waypoint& target,
                     const FollowerConfig& cfg) noexcept {
  const float dx = target.x - current.x;
  const float dy = target.y - current.y;
  const float distance = std::sqrt((dx * dx) + (dy * dy));

  FollowResult result;
  result.distance_remaining = distance;
  result.arrived = distance <= target.arrival_radius;

  if (result.arrived) {
    // Nothing left to steer toward -- hold station. The caller decides
    // what happens next (advance the mission); this function never does.
    result.command.forward_speed = 0.0F;
    result.command.turn_rate = 0.0F;
    return result;
  }

  const float desired_heading = std::atan2(dy, dx);
  const float heading_error = wrap_pi(desired_heading - current.heading_rad);

  result.command.turn_rate =
      std::clamp((heading_error / kPi) * cfg.turn_gain, -cfg.max_turn_rate, cfg.max_turn_rate);

  // cos() of the heading error: full speed when pointed straight at the
  // target, tapering to a stop as the error approaches +-90 degrees, and
  // never negative beyond that -- a waypoint behind the vehicle produces
  // "turn in place, don't reverse," not "drive backwards."
  const float speed_scale = std::max(0.0F, std::cos(heading_error));
  result.command.forward_speed = cfg.cruise_speed * speed_scale;

  return result;
}

}  // namespace nav
