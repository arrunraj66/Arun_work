#pragma once
//
// src/scan_codec.hpp  the ONLY file in this library that mentions both
// lidar::Scan and lidar::proto::LidarScan in the same breath. It lives in
// src/, never include/, so nothing that links this library ever has to know
// protobuf exists  same rule the middleware follows for ZeroMQ.

#include "lidar/scan.hpp"
#include "lidar_scan.pb.h"

namespace lidar {

/// Scan -> wire format. Overwrites everything already in `out`.
void to_proto(const Scan& scan, proto::LidarScan& out);

/// Wire format -> Scan, by value. RVO makes this free; see the line-by-line
/// notes for why returning Scan beats an out-parameter here.
[[nodiscard]] Scan from_proto(const proto::LidarScan& msg);

}  // namespace lidar