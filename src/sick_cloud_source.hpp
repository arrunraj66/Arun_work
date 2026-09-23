#pragma once
#include <cstddef>
#include <memory>
#include <string>

#include "lidar/cloud_source.hpp"

namespace lidar {

class SickCloudSource final : public ICloudSource {
 public:
  struct Config {
    std::string hostname = "192.168.0.1";
    std::string udp_receiver_ip;
    std::string launch_file;
    std::string cloud_name = "cloud_polar_unstructured_fullframe";
    std::string frame_id = "multiscan136";
    std::size_t queue_capacity = 4;
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