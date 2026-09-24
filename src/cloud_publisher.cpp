#include "lidar/cloud_publisher.hpp"

#include <string>
#include <utility>

#include "cloud_codec.hpp"
#include "mw/transport.hpp"

namespace lidar {

struct CloudPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  // ctx declared before publisher: it must outlive the socket. Same
  // one-context-per-publisher reasoning as ScanPublisher::Impl.
  mw::Context ctx;
  mw::Publisher publisher;

  // Reused across calls rather than constructed per cloud -- for a
  // 9000-point frame this matters far more than it did for a 2D Scan.
  proto::LidarPointCloud wire;
};

CloudPublisher::CloudPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

CloudPublisher::~CloudPublisher() = default;
CloudPublisher::CloudPublisher(CloudPublisher&&) noexcept = default;
CloudPublisher& CloudPublisher::operator=(CloudPublisher&&) noexcept = default;

bool CloudPublisher::publish(const PointCloud& cloud) noexcept {
  try {
    to_proto(cloud, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    // Declared noexcept -- see ScanPublisher::publish for why an allocation
    // failure here is counted as one lost cloud, not a reason to terminate.
    return false;
  }
}

std::uint64_t CloudPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& CloudPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace lidar
