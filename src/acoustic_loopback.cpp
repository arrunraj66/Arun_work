#include "mw/acoustic_loopback.hpp"

#include <algorithm>
#include <cstdint>

namespace mw {
namespace {

// Ceiling division for whole nanoseconds: a frame that takes 12.4 ms to
// serialise occupies the channel for 13 ms, not 12. Rounding down would let
// the channel go idle before the last bit has actually left the transducer.
std::int64_t ceil_div(std::int64_t numerator, std::int64_t denominator) noexcept {
  return (numerator + denominator - 1) / denominator;
}

}  // namespace

LoopbackBearer::LoopbackBearer(Config cfg)
    : cfg_(cfg), rng_(cfg.rng_seed), loss_roll_(cfg.loss_probability) {}

std::size_t LoopbackBearer::max_frame_bytes() const noexcept {
  return cfg_.max_frame_bytes;
}

bool LoopbackBearer::offer(std::string_view bytes, Priority priority,
                            Deadline deadline) noexcept {
  ++stats_.offered;

  if (bytes.size() > cfg_.max_frame_bytes) {
    ++stats_.rejected_size;
    return false;
  }

  const std::int64_t bits = static_cast<std::int64_t>(bytes.size()) * 8;
  const std::int64_t serialization_ns = ceil_div(bits * 1'000'000'000LL, cfg_.bits_per_second);
  const std::int64_t arrival_ns = now_ns_ + serialization_ns + cfg_.propagation_ns;

  // A frame this bearer could not deliver in time is refused before it ever
  // touches the channel. A real modem cannot do this  it finds out it was
  // late only when nobody replies. We can, because we know the deadline; a
  // caller with something time-critical should still set priority so the
  // scheduler above this bearer (stage 3 and beyond) offers it first.
  if (arrival_ns > deadline) {
    ++stats_.rejected_late;
    return false;
  }

  // Half-duplex, one frame at a time. This is the property that makes 10 Hz
  // impossible with ten vehicles sharing one channel, independent of anyone's
  // bandwidth math  see the slot-time arithmetic in the swarm material.
  if (now_ns_ < channel_free_at_) {
    ++stats_.rejected_busy;
    return false;
  }

  channel_free_at_ = now_ns_ + serialization_ns;
  ++stats_.accepted;

  // The loss roll happens now, at send time  not at arrival. That is
  // physically honest: whether a frame survives the water is decided by the
  // water, not by whether anyone is later around to notice.
  if (loss_roll_(rng_)) {
    ++stats_.lost;
    return true;   // still accepted: the sender is never told
  }

  in_flight_.push_back(Delivery{std::string(bytes), priority, now_ns_, arrival_ns});
  return true;
}

void LoopbackBearer::advance(std::int64_t ns) noexcept {
  now_ns_ += ns;

  const auto ready = [this](const Delivery& d) { return d.arrives_at_ns <= now_ns_; };

  auto split = std::stable_partition(in_flight_.begin(), in_flight_.end(),
                                      [&](const Delivery& d) { return !ready(d); });
  for (auto it = split; it != in_flight_.end(); ++it) delivered_.push_back(std::move(*it));
  in_flight_.erase(split, in_flight_.end());
}

const std::vector<LoopbackBearer::Delivery>& LoopbackBearer::delivered() const noexcept {
  return delivered_;
}

void LoopbackBearer::drain() noexcept { delivered_.clear(); }

}  // namespace mw