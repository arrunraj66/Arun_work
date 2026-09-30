#include "camera_codec.hpp"

namespace camera {

void to_proto(const Frame& frame, proto::Frame& out) {
  out.set_frame_id(frame.frame_id);
  out.set_stamp_ns(frame.stamp_ns);
  out.set_width(frame.width);
  out.set_height(frame.height);
  out.set_encoding(frame.encoding);
  out.set_data(frame.data.data(), frame.data.size());
}

Frame from_proto(const proto::Frame& wire) {
  Frame frame;
  frame.frame_id = wire.frame_id();
  frame.stamp_ns = wire.stamp_ns();
  frame.width = wire.width();
  frame.height = wire.height();
  frame.encoding = wire.encoding();
  const std::string& bytes = wire.data();
  frame.data.assign(bytes.begin(), bytes.end());
  return frame;
}

}  // namespace camera