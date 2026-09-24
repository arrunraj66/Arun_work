#pragma once
//
// src/cloud_codec.hpp -- the ONLY file that mentions both lidar::PointCloud
// and lidar::proto::LidarPointCloud. Same role scan_codec.hpp plays for
// Scan: lives in src/, so no consumer of lidar::lidar ever sees protobuf.

#include "lidar/point_cloud.hpp"
#include "lidar_point_cloud.pb.h"

namespace lidar {

/// PointCloud -> wire format. Overwrites everything already in `out`.
void to_proto(const PointCloud& cloud, proto::LidarPointCloud& out);

/// Wire format -> PointCloud.
[[nodiscard]] PointCloud from_proto(const proto::LidarPointCloud& msg);

}  // namespace lidar
