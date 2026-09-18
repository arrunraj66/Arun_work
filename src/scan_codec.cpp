#include "scan_codec.hpp"

#include <cstdint>

namespace lidar {

void to_proto(const Scan& scan, proto::LidarScan& out) {
  out.set_frame_id(scan.frame_id);
  out.set_stamp_ns(scan.stamp_ns);
  out.set_scan_time_ns(scan.scan_time_ns);
  out.set_time_increment_ns(scan.time_increment_ns);

  out.set_angle_min(scan.angle_min);
  out.set_angle_max(scan.angle_max);
  out.set_angle_increment(scan.angle_increment);

  out.set_range_min(scan.range_min);
  out.set_range_max(scan.range_max);

  out.set_echo_index(scan.echo_index);
  out.set_echo_count(scan.echo_count);

  out.mutable_ranges()->Clear();
  out.mutable_ranges()->Reserve(static_cast<int>(scan.ranges.size()));
  for (float r : scan.ranges) out.add_ranges(r);

  out.mutable_intensities()->Clear();
  out.mutable_intensities()->Reserve(static_cast<int>(scan.intensities.size()));
  for (float i : scan.intensities) out.add_intensities(i);
}

Scan from_proto(const proto::LidarScan& msg) {
  Scan scan;
  scan.frame_id          = msg.frame_id();
  scan.stamp_ns           = msg.stamp_ns();
  scan.scan_time_ns       = msg.scan_time_ns();
  scan.time_increment_ns  = msg.time_increment_ns();

  scan.angle_min           = msg.angle_min();
  scan.angle_max           = msg.angle_max();
  scan.angle_increment     = msg.angle_increment();

  scan.range_min           = msg.range_min();
  scan.range_max           = msg.range_max();

  // Narrowing uint32 -> uint8_t. A well-formed picoScan150 message never
  // reports more than 3 echoes, so this never truncates in practice; a
  // corrupt or foreign message could send a larger value, and this line
  // silently keeps only its low 8 bits rather than rejecting it. Good enough
  // for Step 2  a real validity check belongs wherever this library first
  // decides to distrust its input, which is not yet.
  scan.echo_index = static_cast<std::uint8_t>(msg.echo_index());
  scan.echo_count = static_cast<std::uint8_t>(msg.echo_count());

  scan.ranges.assign(msg.ranges().begin(), msg.ranges().end());
  scan.intensities.assign(msg.intensities().begin(), msg.intensities().end());

  return scan;
}

}  // namespace lidar