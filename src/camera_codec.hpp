#pragma once

#include "camera/frame.hpp"

#include "camera_frame.pb.h"

namespace camera {

void to_proto(const Frame& frame, proto::Frame& out);
[[nodiscard]] Frame from_proto(const proto::Frame& wire);

}  // namespace camera