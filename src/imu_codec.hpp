#pragma once
//
// src/imu_codec.hpp -- the ONLY file that mentions both imu::Imu and
// imu::proto::Imu. Same role nav_codec.hpp/scan_codec.hpp play for their
// types: lives in src/, so no consumer of imu::imu ever sees protobuf.

#include "imu/imu.hpp"
#include "imu.pb.h"

namespace imu {

/// Imu -> wire format. Overwrites everything already in `out`.
void to_proto(const Imu& msg, proto::Imu& out);

/// Wire format -> Imu.
[[nodiscard]] Imu from_proto(const proto::Imu& wire);

}  // namespace imu
