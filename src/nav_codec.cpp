#include "nav_codec.hpp"

namespace nav {

void to_proto(const NavCommand& cmd, proto::NavCommand& out) {
  out.set_stamp_ns(cmd.stamp_ns);
  out.set_forward_speed(cmd.forward_speed);
  out.set_turn_rate(cmd.turn_rate);
  out.set_obstacle_detected(cmd.obstacle_detected);
}

NavCommand from_proto(const proto::NavCommand& msg) {
  NavCommand cmd;
  cmd.stamp_ns = msg.stamp_ns();
  cmd.forward_speed = msg.forward_speed();
  cmd.turn_rate = msg.turn_rate();
  cmd.obstacle_detected = msg.obstacle_detected();
  return cmd;
}

}  // namespace nav
