#pragma once
//
// src/sick_cloud_source.hpp — SickCloudSource, the 3D counterpart of
// SickScanSource. Same file, same reasoning, same vendor API -- the
// multiScan136/picoScan150 driver delivers BOTH the flattened 2D polar
// cloud SickScanSource reads and a full multi-layer polar cloud with an
// `elevation` field; which one arrives is entirely a function of which
// cloud_name is requested in Config (SickScanSource asks for
// "cloud_polar_unstructured_fullframe" filtered to a single layer by the
// launch file's default scan config; this class asks for the same field
// set but does NOT throw elevation away).
//
// Lives in src/, not include/lidar/, for the identical reason
// sick_scan_source.hpp does: this is the only place besides
// sick_scan_source.hpp that this library mentions sick_scan_xd's vendor
// header. Nobody who links lidar::lidar needs to know it exists.

#include <cstddef>
#include <memory>
#include <string>

#include "lidar/cloud_source.hpp"

namespace lidar {

/// Wraps sick_scan_xd's C API behind ICloudSource, reading the elevation
/// field SickScanSource discards. Construction/destruction semantics are
/// identical to SickScanSource: construction IS the connection, destruction
/// sends the stop command. See sick_scan_source.hpp for the full reasoning
/// -- it applies here unchanged.
class SickCloudSource final : public ICloudSource {
 public:
  struct Config {
    /// The sensor's own IP address (its SOPAS control port 2111). Same
    /// field, same meaning, as SickScanSource::Config::hostname.
    std::string hostname = "192.168.0.1";

    /// THIS machine's IP address on the sensor's subnet. Required -- see
    /// SickScanSource::Config::udp_receiver_ip for why.
    std::string udp_receiver_ip;

    /// Path to sick_scan_xd's launch file for THIS sensor family:
    /// sick_multiscan.launch for the multiScan136, sick_picoscan.launch for
    /// the picoScan150. Required.
    std::string launch_file;

    /// Same meaning as SickScanSource::Config::cloud_name -- one fullframe
    /// cloud, so poll_cloud() returns one PointCloud per revolution rather
    /// than an interleaved mix of segments and fullframes.
    std::string cloud_name = "cloud_polar_unstructured_fullframe";

    /// Becomes every PointCloud::frame_id this source produces.
    std::string frame_id = "multiscan136";

    /// How many clouds to buffer between poll_cloud() calls. Same
    /// bounded/newest-wins policy as SickScanSource::Config::queue_capacity
    /// -- a full 3D frame is far larger than a 2D scan, so the default is
    /// deliberately smaller (a live consumer should be draining this close
    /// to every revolution; buffering many multi-thousand-point frames is
    /// a lot of memory for data that is about to be superseded anyway).
    std::size_t queue_capacity = 4;

    /// The sensor's own measurement limits, copied into every PointCloud.
    /// Defaults are the multiScan136's specified range; not read back from
    /// the device.
    float range_min = 0.05F;
    float range_max = 120.0F;

    /// UDP ports this sensor pushes to. 0 means "leave the launch file's
    /// own default alone" -- sick_multiscan.launch defaults to 2115/7503,
    /// the SAME defaults sick_picoscan.launch uses. Running a picoScan150
    /// (2D) and a multiScan136 (3D) at the same time means one of them
    /// MUST override these, or the second sensor's driver binds a UDP port
    /// the first one already owns and silently receives nothing (or the
    /// wrong sensor's data). Set both to something distinct -- e.g. 2125
    /// and 7513 -- whenever this runs alongside SickScanSource.
    int udp_port = 0;
    int imu_udp_port = 0;
  };

  /// Connects and enables streaming. Throws std::runtime_error if the
  /// configuration is incomplete or the sensor can't be reached -- same
  /// "failed connection is a failed construction" rule as SickScanSource.
  explicit SickCloudSource(Config cfg);
  ~SickCloudSource() override;

  SickCloudSource(const SickCloudSource&) = delete;
  SickCloudSource& operator=(const SickCloudSource&) = delete;
  SickCloudSource(SickCloudSource&&) noexcept;
  SickCloudSource& operator=(SickCloudSource&&) noexcept;

  [[nodiscard]] bool poll_cloud(PointCloud& out, int timeout_ms) noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar
