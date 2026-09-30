#pragma once
//
// nav/pose_publisher.hpp — PosePublisher, same shape as nav::NavPublisher.
// Built on mw::Publisher; hides that a VehiclePose has to become bytes
// before it can leave this process. Meant to be called every simulation
// tick, same rate as the IMU/lidar/camera publishers already are.

#include <cstdint>
#include <memory>
#include <string>

#include "nav/vehicle_pose.hpp"

namespace nav {

/// Publishes VehiclePoses to anyone who subscribes -- in practice, a
/// waypoint_follow_main process. Same fan-out, fire-and-forget contract as
/// every other Publisher in this project.
class PosePublisher {
 public:
  /// `endpoint` is a ZeroMQ address, which this publisher BINDS.
  /// `topic` defaults to "nav.pose" -- distinct from "nav.cmd" and
  /// "nav.mission" so nothing downstream can mix them up.
  explicit PosePublisher(std::string endpoint, std::string topic = "nav.pose");
  ~PosePublisher();

  PosePublisher(const PosePublisher&) = delete;
  PosePublisher& operator=(const PosePublisher&) = delete;
  PosePublisher(PosePublisher&&) noexcept;
  PosePublisher& operator=(PosePublisher&&) noexcept;

  /// Serialises `pose` and sends it. Returns false if the send queue was
  /// full and the pose was dropped.
  [[nodiscard]] bool publish(const VehiclePose& pose) noexcept;

  /// How many poses this publisher has dropped since it was created.
  [[nodiscard]] std::uint64_t dropped() const noexcept;

  /// The topic this publisher sends under.
  [[nodiscard]] const std::string& topic() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav
