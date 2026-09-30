#include "imu_codec.hpp"

namespace imu {

namespace {

void to_proto(const Vec3& v, proto::Vec3& out) {
  out.set_x(v.x);
  out.set_y(v.y);
  out.set_z(v.z);
}

Vec3 from_proto(const proto::Vec3& wire) {
  Vec3 v;
  v.x = wire.x();
  v.y = wire.y();
  v.z = wire.z();
  return v;
}

void to_proto(const Quaternion& q, proto::Quaternion& out) {
  out.set_x(q.x);
  out.set_y(q.y);
  out.set_z(q.z);
  out.set_w(q.w);
}

Quaternion from_proto(const proto::Quaternion& wire) {
  Quaternion q;
  q.x = wire.x();
  q.y = wire.y();
  q.z = wire.z();
  q.w = wire.w();
  return q;
}

}  // namespace

void to_proto(const Imu& msg, proto::Imu& out) {
  out.set_frame_id(msg.frame_id);
  out.set_stamp_ns(msg.stamp_ns);
  out.set_orientation_valid(msg.has_orientation);
  to_proto(msg.orientation, *out.mutable_orientation());
  to_proto(msg.angular_velocity, *out.mutable_angular_velocity());
  to_proto(msg.linear_acceleration, *out.mutable_linear_acceleration());
}

Imu from_proto(const proto::Imu& wire) {
  Imu msg;
  msg.frame_id = wire.frame_id();
  msg.stamp_ns = wire.stamp_ns();
  msg.has_orientation = wire.orientation_valid();
  msg.orientation = from_proto(wire.orientation());
  msg.angular_velocity = from_proto(wire.angular_velocity());
  msg.linear_acceleration = from_proto(wire.linear_acceleration());
  return msg;
}

}  // namespace imu
