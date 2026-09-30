#include "camera/camera_publisher.hpp"

#include <string>

#include <utility>

#include "camera_codec.hpp"

#include "mw/transport.hpp"

namespace camera {

struct CameraPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;
  mw::Context ctx;
  mw::Publisher publisher;
  proto::Frame wire;
};

CameraPublisher::CameraPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

CameraPublisher::~CameraPublisher() = default;
CameraPublisher::CameraPublisher(CameraPublisher&&) noexcept = default;
CameraPublisher& CameraPublisher::operator=(CameraPublisher&&) noexcept = default;

bool CameraPublisher::publish(const Frame& frame) noexcept {
  try {
    to_proto(frame, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    return false;
  }
}

std::uint64_t CameraPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }
const std::string& CameraPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace camera