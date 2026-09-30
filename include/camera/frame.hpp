#pragma once
//
// camera/frame.hpp -- the one type every consumer of the camera module
// actually wants, same "plain struct, standard library only" rule as
// imu::Imu and lidar::Scan: no protobuf, no ZeroMQ, no OpenCV.

#include <cstdint>

#include <string>

#include <vector>

namespace camera {

struct Frame {
  std::string frame_id;
  std::int64_t stamp_ns = 0;

  std::int32_t width = 0;
  std::int32_t height = 0;

  // "jpeg", "png" -- whatever cv::imencode's own extension argument was
  // (encoding == the extension used, without the leading dot).
  std::string encoding;

  // Already-encoded bytes (e.g. what cv::imencode(".jpg", ...) produced).
  std::vector<std::uint8_t> data;
};

}  // namespace camera