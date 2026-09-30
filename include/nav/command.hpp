#pragma once
//
// nav/command.hpp -- the one type every consumer of the nav module actually
// wants: a steering decision, in the same "plain struct, no protobuf, no
// ZeroMQ" spirit as lidar::Scan and lidar::PointCloud.
//
// This is a DECISION, not a physics command. It deliberately does not say
// "metres per second" or "radians per second" -- there is no real thruster
// driver yet, and the Webots controller currently moves the vehicle by
// teleporting its translation/rotation fields once per simulation step, not
// by simulating forces. So both fields are normalised to [-1, 1]: "how much
// of a full step, this tick" -- exactly the same shape as one keyboard
// press already produces (movementStep * 1, rotationStep * 1). Whoever
// applies a NavCommand scales it by whatever a full step means to them.
// When a real thruster driver exists, it gets its own, differently-scaled
// consumer of this same struct -- nothing here has to change for that.

#include <cstdint>

namespace nav {

struct NavCommand {
  std::int64_t stamp_ns = 0;  // when this decision was made, ns since epoch

  // Both normalised to [-1, 1]. forward_speed: -1 = full reverse (not
  // currently produced by NavAvoider -- see avoider.hpp), 0 = stopped,
  // +1 = full cruise speed. turn_rate: positive = turn left (matches the
  // Webots controller's own convention: LEFT key increases yaw), negative
  // = turn right.
  float forward_speed = 0.0F;
  float turn_rate = 0.0F;

  // True whenever this decision was influenced by a nearby obstacle --
  // i.e. the front sector was inside slow_distance or closer. A diagnostic
  // flag, not used to decide anything downstream; useful on a status
  // display or in a log to see when the vehicle is actively avoiding
  // something versus cruising freely.
  bool obstacle_detected = false;
};

}  // namespace nav
