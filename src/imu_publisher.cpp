#include "imu/imu_publisher.hpp"

#include <string>
#include <utility>

#include "imu_codec.hpp"
#include "mw/transport.hpp"

namespace imu {

struct ImuPublisher::Impl {
  Impl(std::string ep, std::string t)
      : endpoint(std::move(ep)), topic(std::move(t)), publisher(ctx, endpoint) {}

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  std::string endpoint;
  std::string topic;

  mw::Context ctx;
  mw::Publisher publisher;

  proto::Imu wire;
};

ImuPublisher::ImuPublisher(std::string endpoint, std::string topic)
    : impl_(std::make_unique<Impl>(std::move(endpoint), std::move(topic))) {}

ImuPublisher::~ImuPublisher() = default;
ImuPublisher::ImuPublisher(ImuPublisher&&) noexcept = default;
ImuPublisher& ImuPublisher::operator=(ImuPublisher&&) noexcept = default;

bool ImuPublisher::publish(const Imu& msg) noexcept {
  try {
    to_proto(msg, impl_->wire);
    const std::string bytes = impl_->wire.SerializeAsString();
    return impl_->publisher.publish(impl_->topic, bytes);
  } catch (...) {
    return false;
  }
}

std::uint64_t ImuPublisher::dropped() const noexcept { return impl_->publisher.dropped(); }

const std::string& ImuPublisher::topic() const noexcept { return impl_->topic; }

}  // namespace imu
