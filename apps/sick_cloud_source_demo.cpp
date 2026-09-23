// The 3D counterpart of sick_scan_source_demo.cpp -- same shape, same
// purpose: prove SickCloudSource connects, streams, and disconnects,
// before it's wired into anything bigger (a publisher, a recorder).
//
// This program also settles the one thing left unconfirmed since Step 3:
// whether the vendor's elevation angle is positive-up or positive-down.
// Every frame it prints the xyz of the point with the LOWEST elevation
// (the beam aimed most steeply down) and the HIGHEST elevation (aimed most
// steeply up), computed with lidar::to_xyz(). Point the sensor across open
// floor and ceiling/sky and read the z sign:
//   - if to_xyz() is right, the lowest-elevation point has NEGATIVE z
//     (floor, below the sensor) and the highest-elevation point has
//     POSITIVE z (ceiling/sky, above the sensor).
//   - if it's backwards, they'll be swapped -- and the fix is a one-line
//     sign flip in lidar/point_cloud.hpp's to_xyz(), nowhere else, because
//     every consumer of PointCloud goes through that one function.
//
// Usage:
//   ./sick_cloud_source_demo <launch_file> <sensor_ip> <this_machine_ip>
//
// Example (multiScan136, the combination verified in Step 1):
//   ./sick_cloud_source_demo ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch
//                            192.168.12.223 192.168.12.240
//
// Ctrl+C handling: sick_scan_xd is a ROS driver ported to run without ROS,
// and its init sequence (SickScanApiInitByLaunchfile) installs its own
// SIGINT handler internally, the same way a ROS node expects ros::spin()
// to notice ros::ok() go false -- but nothing here calls into ROS, so that
// internal handler has nobody to signal. The practical effect: a plain
// while(true) loop with no handler of our own does not reliably stop on
// Ctrl+C once the SDK has finished connecting.
//
// Fixed the same way camera_publisher_demo.cpp handles it: our own
// std::atomic<bool> flag, set from our own SIGINT handler, checked once per
// loop iteration -- and installed AFTER the connect (after `source` is
// constructed) specifically so ours is the handler left in place, not
// whatever the SDK's own init put there first.

#include "sick_cloud_source.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <exception>

#include "lidar/point_cloud.hpp"

namespace {
std::atomic<bool> g_stop_requested{false};
void handle_sigint(int /*signal_number*/) { g_stop_requested.store(true); }
}  // namespace

int main(int argc, char** argv) {
if (argc < 4) {
std::fprintf(stderr,
"usage: %s <launch_file> <sensor_ip> <this_machine_ip>\n"
"  e.g. %s ~/sick_scan_ws/sick_scan_xd/launch/sick_multiscan.launch"
" 192.168.12.223 192.168.12.240\n"
"  (find this machine's IP on the sensor's subnet with: ip -4 addr show)\n",
argv[0], argv[0]);
return 1;
}

lidar::SickCloudSource::Config cfg;
cfg.launch_file = argv[1];
cfg.hostname = argv[2];
cfg.udp_receiver_ip = argv[3];

std::printf("connecting to 3D lidar at %s (scan data -> %s) ...\n", cfg.hostname.c_str(),
cfg.udp_receiver_ip.c_str());

try {
lidar::SickCloudSource source(cfg);  // <-- the connect happens on this line

// Installed AFTER the connect, on purpose -- see the comment at the top
// of this file for why installing it any earlier would not stick.
std::signal(SIGINT, handle_sigint);

std::printf("connected. printing clouds (Ctrl+C to stop) ...\n");

lidar::PointCloud cloud;
int cloud_count = 0;
while (!g_stop_requested.load()) {
if (!source.poll_cloud(cloud, /*timeout_ms=*/1000)) {
std::printf("  ... no cloud within 1000 ms, still waiting\n");
continue;
}
++cloud_count;
std::printf("cloud #%d: frame_id=%s stamp_ns=%lld points=%zu scan_time_ms=%.1f\n",
cloud_count, cloud.frame_id.c_str(), static_cast<long long>(cloud.stamp_ns),
cloud.size(), static_cast<double>(cloud.scan_time_ns) / 1.0e6);

if (cloud.size() > 0) {
std::size_t lowest = 0;
std::size_t highest = 0;
for (std::size_t i = 1; i < cloud.size(); ++i) {
if (cloud.elevations[i] < cloud.elevations[lowest]) lowest = i;
if (cloud.elevations[i] > cloud.elevations[highest]) highest = i;
}
   	
const lidar::Point3 p_low = lidar::to_xyz(cloud, lowest);
const lidar::Point3 p_high = lidar::to_xyz(cloud, highest);
std::printf(
   "  lowest  elevation=%.3f rad  range=%.3f m  -> xyz=(%.3f, %.3f, %.3f)"
   "   [expect z<0 if pointed at open floor]\n",
 static_cast<double>(cloud.elevations[lowest]),
   static_cast<double>(cloud.ranges[lowest]), static_cast<double>(p_low.x),
static_cast<double>(p_low.y), static_cast<double>(p_low.z));
   std::printf(
"  highest elevation=%.3f rad  range=%.3f m  -> xyz=(%.3f, %.3f, %.3f)"
   "   [expect z>0 if pointed at open ceiling/sky]\n",
    static_cast<double>(cloud.elevations[highest]),
  static_cast<double>(cloud.ranges[highest]), static_cast<double>(p_high.x),
  static_cast<double>(p_high.y), static_cast<double>(p_high.z));
 }
    }
 std::printf("\nsick_cloud_source_demo: stopping -- %d clouds received\n", cloud_count);
// `source` goes out of scope here -- that IS the disconnect (stop
  // command sent, resources released), same as SickScanSource.
} catch (const std::exception& e) {
 std::fprintf(stderr, "failed to connect: %s\n", e.what());
  return 1;
   }
 }
