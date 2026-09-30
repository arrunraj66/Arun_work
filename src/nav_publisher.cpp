#include "nav/nav_publisher.hpp"

#include <string>
#include <utility>

#include "mw/transport.hpp"
#include "nav_codec.hpp"

namespace nav {

struct NavPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  mw::Context ctx;
  mw::Publisher publisher;

  proto::NavCommand wire;
};

NavPublisher::NavPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

NavPublisher::~NavPublisher() = default;
NavPublisher::NavPublisher(NavPublisher&&) noexcept = default;
NavPublisher& NavPublisher::operator=(NavPublisher&&) noexcept = default;

bool NavPublisher::publish(const NavCommand& cmd) noexcept {
  try {
    to_proto(cmd, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    return false;
  }
}

std::uint64_t NavPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& NavPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace nav
