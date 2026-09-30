#include "nav/pose_publisher.hpp"

#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "waypoint_codec.hpp"

namespace nav {

struct PosePublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  mw::Context ctx;
  mw::Publisher publisher;

  proto::VehiclePose wire;
};

PosePublisher::PosePublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

PosePublisher::~PosePublisher() = default;
PosePublisher::PosePublisher(PosePublisher&&) noexcept = default;
PosePublisher& PosePublisher::operator=(PosePublisher&&) noexcept = default;

bool PosePublisher::publish(const VehiclePose& pose) noexcept {
  try {
    to_proto(pose, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    return false;
  }
}

std::uint64_t PosePublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& PosePublisher::topic() const noexcept { return impl_->topic; }

}  // namespace nav
