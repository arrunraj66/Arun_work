#include "cloud_codec.hpp"

#include <vector>

#include <google/protobuf/repeated_field.h>

namespace lidar {

namespace {

// One helper instead of four copies of the same three lines. Clear() first
// because `out` is reused frame after frame (see CloudPublisher later) and
// must never keep the previous frame's points.
void copy_floats(const std::vector<float>& from, google::protobuf::RepeatedField<float>* to) {
  to->Clear();
  to->Reserve(static_cast<int>(from.size()));
  for (float v : from) to->Add(v);
}

}  // namespace

void to_proto(const PointCloud& cloud, proto::LidarPointCloud& out) {
  out.set_frame_id(cloud.frame_id);
  out.set_stamp_ns(cloud.stamp_ns);
  out.set_scan_time_ns(cloud.scan_time_ns);
  out.set_range_min(cloud.range_min);
  out.set_range_max(cloud.range_max);

  copy_floats(cloud.ranges, out.mutable_ranges());
  copy_floats(cloud.azimuths, out.mutable_azimuths());
  copy_floats(cloud.elevations, out.mutable_elevations());
  copy_floats(cloud.intensities, out.mutable_intensities());
}

PointCloud from_proto(const proto::LidarPointCloud& msg) {
  PointCloud cloud;
  cloud.frame_id     = msg.frame_id();
  cloud.stamp_ns     = msg.stamp_ns();
  cloud.scan_time_ns = msg.scan_time_ns();
  cloud.range_min    = msg.range_min();
  cloud.range_max    = msg.range_max();

  cloud.ranges.assign(msg.ranges().begin(), msg.ranges().end());
  cloud.azimuths.assign(msg.azimuths().begin(), msg.azimuths().end());
  cloud.elevations.assign(msg.elevations().begin(), msg.elevations().end());
  cloud.intensities.assign(msg.intensities().begin(), msg.intensities().end());
  return cloud;
}

}  // namespace lidar
