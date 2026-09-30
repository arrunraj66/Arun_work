#pragma once
//
// nav/avoider.hpp -- Navigation Step 1: pure reactive obstacle avoidance.
// No mission, no waypoints, no path planning -- those are explicitly a
// later step. This is a Braitenberg-style reactive controller: it looks at
// the nearest obstacle in three directions right now and reacts, with no
// memory of anything before this tick and no notion of "where am I trying
// to go." That scope is deliberate, the same way Scan started as just a
// struct before anything else got layered on top of it.
//
// decide() is a pure function: no I/O, no ZeroMQ, no lidar::Scan even --
// it takes plain distances in and returns a plain NavCommand out. That is
// what makes it trivial to unit test (feed it numbers, check the sign and
// magnitude of what comes back) and reusable regardless of what the "front
// sensor" turns out to be later (a lidar today, maybe a sonar or a camera
// depth estimate tomorrow) -- something else's job is to turn a Scan into
// a SectorRanges in the first place.

#include "nav/command.hpp"

namespace nav {

/// The nearest valid obstacle distance in three directions, metres.
/// A direction with nothing in range (or no reading at all yet) should be
/// set to an effectively-infinite value -- std::numeric_limits<float>::
/// infinity() is the intended sentinel, not 0 or -1, so "nothing there" can
/// never be mistaken for "obstacle at zero range."
struct SectorRanges {
  float front = 0.0F;
  float left = 0.0F;
  float right = 0.0F;
};

/// Tunable thresholds. Defaults are normalised-step scale, matching one
/// Webots controller keyboard tick (see command.hpp) -- not tied to any
/// particular sensor's real range_min/range_max.
struct AvoiderConfig {
  // Front obstacle at or inside this distance: stop translating, turn in
  // place. Must be < slow_distance.
  float stop_distance = 0.5F;

  // Front obstacle at or inside this distance (but outside stop_distance):
  // slow down and steer, proportionally to how close it is.
  float slow_distance = 2.5F;

  // forward_speed when the front sector is clear (front > slow_distance).
  float cruise_speed = 1.0F;

  // The largest |turn_rate| this function will ever produce.
  float max_turn_rate = 1.0F;
};

/// The whole decision, in one call. See avoider.cpp for the three-tier
/// logic (clear / slowing / stopped-and-turning) and the worked reasoning
/// behind each threshold.
[[nodiscard]] NavCommand decide(const SectorRanges& sectors, const AvoiderConfig& cfg = {}) noexcept;

}  // namespace nav
