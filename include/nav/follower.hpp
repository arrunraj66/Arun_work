#pragma once
//
// nav/follower.hpp -- Navigation Step 2: waypoint following. Same
// "pure function, no I/O" shape as nav::decide() in avoider.hpp: takes a
// VehiclePose and a single target Waypoint in, returns a NavCommand out,
// nothing else. No ZeroMQ, no protobuf, no notion of a whole mission or
// which waypoint index is current -- that bookkeeping belongs to whoever
// calls this every tick (see waypoint_follow_main.cpp), the same way
// avoider.hpp leaves "what SectorRanges actually is this tick" to its own
// caller.

#include "nav/command.hpp"
#include "nav/vehicle_pose.hpp"
#include "nav/waypoint.hpp"

namespace nav {

struct FollowerConfig {
  // forward_speed produced when heading straight at the target, same
  // normalised [-1, 1] scale as AvoiderConfig::cruise_speed.
  float cruise_speed = 1.0F;

  // The largest |turn_rate| this function will ever produce.
  float max_turn_rate = 1.0F;

  // Proportional gain on heading error -> turn_rate. 1.0 means a full
  // +-180 degree heading error alone saturates turn_rate to
  // +-max_turn_rate; raise it to turn harder for smaller errors.
  float turn_gain = 1.0F;
};

struct FollowResult {
  NavCommand command;

  // True once `current` is within `target.arrival_radius` of `target`.
  // The caller (not this function) decides what "arrived" means for the
  // mission as a whole -- advance to the next waypoint, stop, loop back to
  // the first.
  bool arrived = false;

  // Straight-line distance remaining to `target`, metres. For
  // logging/telemetry -- not needed to interpret `arrived`.
  float distance_remaining = 0.0F;
};

/// One steering decision toward one waypoint. Heading control is
/// proportional (turn toward the target, harder the more off-heading you
/// are); forward speed tapers to zero as heading error approaches +-90
/// degrees and beyond, the same "don't drive forward while turning in
/// place" idea AvoiderConfig's stop tier uses -- so the vehicle turns to
/// face a waypoint behind it before committing to forward motion, rather
/// than driving backwards or arcing wide.
[[nodiscard]] FollowResult follow(const VehiclePose& current, const Waypoint& target,
                                   const FollowerConfig& cfg = {}) noexcept;

}  // namespace nav
