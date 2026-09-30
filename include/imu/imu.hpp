#pragma once
//
// imu/imu.hpp -- the one type every consumer of the imu module actually
// wants, same "plain struct, standard library only" rule as lidar::Scan
// and lidar::PointCloud: no protobuf, no ZeroMQ, no vendor SDK.
//
// Field layout mirrors the SICK driver's own SickScanImuMsg (itself,
// per the vendor's own comment, "equivalent to ros sensor_msgs::Imu") --
// not because we depend on that struct, but so the adapter that reads it
// is a straight field-by-field copy with nothing to get subtly wrong in
// translation. Any other IMU source (a different vendor, a simulated one
// in Webots) fills this same struct its own way.

#include <cstdint>
#include <string>

namespace imu {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;  // identity (no rotation) by default, not all-zero
};

struct Imu {
  std::string frame_id;
  std::int64_t stamp_ns = 0;

  // Not every IMU (or every driver) fuses its own orientation estimate --
  // some report only raw gyro/accel and leave fusion to the consumer. The
  // ROS convention this struct's source (SickScanImuMsg) says it follows
  // is orientation_covariance[0] < 0 meaning "orientation not provided" --
  // has_orientation is that fact, checked explicitly by whatever adapter
  // fills this struct, not assumed. A consumer must check it before
  // trusting `orientation`; an all-zero-or-garbage quaternion must never
  // be read as real data just because the field exists.
  bool has_orientation = false;
  Quaternion orientation;

  Vec3 angular_velocity;     // rad/s, about x/y/z
  Vec3 linear_acceleration;  // m/s^2, about x/y/z -- INCLUDES gravity (a
                              // stationary, level IMU reads ~9.81 on
                              // whichever axis is "up"; this is the same
                              // convention ROS sensor_msgs/Imu uses, and
                              // it is what makes an accelerometer usable
                              // for finding "up" at all -- see the
                              // leveling/fusion notes this ships with)
};

}  // namespace imu
