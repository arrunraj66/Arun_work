#include "nav/avoider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav {

namespace {

// left/right are compared as a normalised difference rather than a raw
// subtraction so "5m vs 3m" and "50m vs 30m" steer the same amount -- what
// matters is which side is relatively more open, not the absolute gap.
// Guards against inf-inf (NaN) and 0/0 when neither side has a reading.
[[nodiscard]] float side_bias(float left, float right) noexcept {
  const bool left_finite = std::isfinite(left);
  const bool right_finite = std::isfinite(right);

  if (!left_finite && !right_finite) {
    return 0.0F;  // neither side has read anything yet -- no basis to steer
  }
  if (left_finite && !right_finite) {
    return -1.0F;  // right is wide open, left is the constrained side -> steer right
  }
  if (!left_finite && right_finite) {
    return 1.0F;  // mirror of the above -> steer left
  }

  const float sum = left + right;
  if (sum <= std::numeric_limits<float>::epsilon()) {
    return 0.0F;  // both essentially zero -- degenerate, no safe bias to give
  }
  // >0 means left has more room (steer left); <0 means right does.
  return (left - right) / sum;
}

}  // namespace

NavCommand decide(const SectorRanges& sectors, const AvoiderConfig& cfg) noexcept {
  NavCommand cmd;
  cmd.obstacle_detected = sectors.front <= cfg.slow_distance;

  // Tier 3: too close to keep moving forward at all. Stop translating,
  // turn in place toward whichever side currently has more room. This is
  // the only tier that ignores forward_speed entirely -- there is no safe
  // "slow forward" once the front sector is this close.
  if (sectors.front <= cfg.stop_distance) {
    cmd.forward_speed = 0.0F;
    const float bias = side_bias(sectors.left, sectors.right);
    // A genuine tie (bias == 0, e.g. both sides unknown) still has to pick
    // a direction rather than sit still and do nothing -- left is chosen
    // arbitrarily but consistently, so behaviour is deterministic and
    // testable rather than "whatever floating point happened to do."
    cmd.turn_rate = (bias >= 0.0F) ? cfg.max_turn_rate : -cfg.max_turn_rate;
    return cmd;
  }

  // Tier 2: an obstacle is ahead but not urgent yet. Slow down in
  // proportion to how far into the [stop_distance, slow_distance] band the
  // front reading has come, and start steering toward the more open side.
  if (sectors.front <= cfg.slow_distance) {
    const float band = cfg.slow_distance - cfg.stop_distance;
    // band <= 0 would mean a misconfigured AvoiderConfig (slow_distance
    // must be > stop_distance) -- guard rather than divide by ~0.
    const float urgency =
        (band > std::numeric_limits<float>::epsilon())
            ? std::clamp((cfg.slow_distance - sectors.front) / band, 0.0F, 1.0F)
            : 1.0F;

    cmd.forward_speed = cfg.cruise_speed * (1.0F - urgency);
    cmd.turn_rate = std::clamp(side_bias(sectors.left, sectors.right) * cfg.max_turn_rate,
                                -cfg.max_turn_rate, cfg.max_turn_rate);
    return cmd;
  }

  // Tier 1: clear ahead. Go straight -- no side-steering here on purpose;
  // reacting to the side sensors only when actively avoiding something in
  // front keeps this step's behaviour simple enough to fully reason about,
  // rather than a vehicle that wanders off a straight line in open water
  // because a wall happened to be closer on one side.
  cmd.forward_speed = cfg.cruise_speed;
  cmd.turn_rate = 0.0F;
  return cmd;
}

}  // namespace nav
