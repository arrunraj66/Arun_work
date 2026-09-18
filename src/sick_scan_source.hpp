#pragma once
//
// src/sick_scan_source.hpp  SickScanSource, one concrete IScanSource.
//
// Lives in src/, not include/lidar/, for the exact reason scan_codec.hpp
// does: it is the only place this library ever mentions sick_scan_xd's
// vendor header, and nothing in include/lidar/ may do that. Nobody who
// links lidar::lidar needs to know sick_scan_xd exists  only our own
// sick_scan_source_demo.cpp (the process that actually owns the sensor)
// constructs one of these.
#include <cstddef>
#include <memory>
#include <string>

#include "lidar/source.hpp"

namespace lidar {

/// Wraps sick_scan_xd's C API behind IScanSource.
///
/// Construction IS the connection trigger; destruction IS the disconnect.
/// There is no separate connect()/start() method, deliberately: the object's
/// lifetime is the connection's lifetime, so "is it connected" is never a
/// question a caller has to ask or a state that can drift out of sync.
class SickScanSource final : public IScanSource {
 public:
  struct Config {
    /// The picoScan150's own IP address (its SOPAS control port 2111).
    std::string hostname = "192.168.0.1";

    /// THIS machine's IP address on the sensor's subnet. Required, with no
    /// usable default: the picoScan150 does not answer scan data back down
    /// the TCP connection, it *pushes* UDP datagrams to an address we hand
    /// it during the start handshake. Leave this empty and the driver fails
    /// with `can't split ip address "" into 4 tokens`  confirmed against
    /// the real unit. Find it with: ip -4 addr show
    std::string udp_receiver_ip;

    /// Path to sick_scan_xd's own sick_picoscan.launch. Required.
    ///
    /// We load the vendor's launch file and override individual parameters
    /// rather than assembling a parameter string from scratch. That file
    /// sets scanner_type, the SOPAS access-level password, echo/angle
    /// filters, UDP ports and a dozen other things correctly for this
    /// sensor family; hand-assembling an equivalent means silently
    /// reinventing all of it. This is also the only invocation style the
    /// vendor's own test scripts use, so it is the one that gets tested
    /// upstream.
    std::string launch_file;

    /// Which of the launch file's predefined point clouds to enable. The
    /// default is deliberately ONE cloud, overriding the launch file's
    /// default list of five.
    ///
    /// This matters more than it looks: sick_scan_xd routes every enabled
    /// cloud with polar coordinates to the same polar callback that
    /// poll_scan() reads (ros_msgpack_publisher.cpp, publishPointCloud2Msg).
    /// With the launch file's defaults, three separate clouds qualify  two
    /// of them *segments* (fractions of a revolution) and one a fullframe 
    /// so poll_scan() would receive an interleaved mix of partial and
    /// complete scans with no way to tell them apart. Naming one fullframe
    /// cloud means one Scan out per revolution, which is what lidar::Scan
    /// is defined to be.
    std::string cloud_name = "cloud_polar_unstructured_fullframe";

    /// Becomes every Scan::frame_id this source produces.
    std::string frame_id = "picoscan150";
    /// How many scans to buffer between poll_scan() calls. Scans arrive on
    /// a driver thread and are queued; if a caller polls slower than the
    /// sensor produces, the OLDEST queued scan is dropped rather than
    /// letting the queue grow without bound. For a live sensor the newest
    /// data is what matters, and an unbounded queue would turn a slow
    /// consumer into a memory leak.
    std::size_t queue_capacity = 8;
    /// The sensor's own measurement limits, copied into every Scan so that
    /// is_valid() has something meaningful to compare against. Defaults are
    /// the picoScan150's specified range; they are not read back from the
    /// device, so change them here if the datasheet disagrees.
    float range_min = 0.05F;
    float range_max = 120.0F;
  };

  /// Connects and enables streaming. Throws std::runtime_error if the
  /// configuration is incomplete or the sensor can't be reached.
  explicit SickScanSource(Config cfg);
  ~SickScanSource() override;

  SickScanSource(const SickScanSource&) = delete;
  SickScanSource& operator=(const SickScanSource&) = delete;
  SickScanSource(SickScanSource&&) noexcept;
  SickScanSource& operator=(SickScanSource&&) noexcept;

  [[nodiscard]] bool poll_scan(Scan& out, int timeout_ms) noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lidar