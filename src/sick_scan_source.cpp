#include "sick_scan_source.hpp"

#include <sick_scan_xd_api/sick_scan_api.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>

namespace lidar {

namespace {

// Finds a named field's byte offset inside one point record, or -1 if this
// cloud doesn't carry it.
int find_field_offset(const SickScanPointFieldArray& fields, const char* name) {
  for (std::uint64_t i = 0; i < fields.size; ++i) {
    if (std::strcmp(fields.buffer[i].name, name) == 0) {
      return static_cast<int>(fields.buffer[i].offset);
    }
  }
  return -1;
}

// Intensity is published under the field name "i", not "intensity" --
// ros_msgpack_publisher.cpp:602: PointCloudFieldProperty("i", ...).
int find_intensity_offset(const SickScanPointFieldArray& fields) {
  const int offset = find_field_offset(fields, "i");
  return offset >= 0 ? offset : find_field_offset(fields, "intensity");
}

// memcpy, not reinterpret_cast: `point + offset` isn't guaranteed aligned
// for a float, and an unaligned float read is undefined behaviour.
//
// T must match the field's declared PointField type exactly. For this cloud
// range/azimuth/elevation/i are FLOAT32 and echo is INT8
// (ros_msgpack_publisher.cpp:596-628) -- reading the one-byte echo as
// int32_t would pull in three bytes of whatever follows it.
template <typename T>
T read_field(const std::uint8_t* point, int offset) {
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

// Longest interval still treated as a real revolution time. A 2D LiDAR
// spinning slower than this is not spinning; anything larger is a clock
// discontinuity or a stream gap, where the elapsed wall time says nothing
// useful about how long the scan itself took.
constexpr std::int64_t kMaxPlausibleScanTimeNs = 5'000'000'000LL;  // 5 seconds

}  // namespace

struct SickScanSource::Impl {
  // sick_scan_xd's callback is a plain C function pointer with no user-data
  // argument, so the only way back from "a message arrived on handle H" to
  // "which SickScanSource owns H" is a lookup table. Keyed by handle, so
  // several sources (two sensors) can coexist in one process.
  static std::map<SickScanApiHandle, Impl*>& registry() {
    static std::map<SickScanApiHandle, Impl*> instances;
    return instances;
  }
  static std::mutex& registry_mutex() {
    static std::mutex m;
    return m;
  }

  explicit Impl(Config c) : cfg(std::move(c)) {
    // Fail on an incomplete configuration here, with a message naming the
    // missing field, rather than letting the driver fail several layers
    // down with something like `can't split ip address "" into 4 tokens`.
    if (cfg.hostname.empty()) {
      throw std::runtime_error("SickScanSource: Config::hostname is empty");
    }
    if (cfg.udp_receiver_ip.empty()) {
      throw std::runtime_error(
          "SickScanSource: Config::udp_receiver_ip is empty -- it must be THIS machine's "
          "IP address on the sensor's subnet (find it with: ip -4 addr show). The sensor "
          "pushes scan data to that address over UDP; without it the start handshake fails.");
    }
    if (cfg.launch_file.empty()) {
      throw std::runtime_error(
          "SickScanSource: Config::launch_file is empty -- it must be the path to "
          "sick_scan_xd's launch/sick_picoscan.launch");
    }
    if (cfg.queue_capacity == 0) {
      throw std::runtime_error("SickScanSource: Config::queue_capacity must be at least 1");
    }

    handle = SickScanApiCreate(0, nullptr);
    if (handle == nullptr) {
      throw std::runtime_error("SickScanSource: SickScanApiCreate returned null");
    }

    // Publish ourselves in the registry BEFORE registering the callback, so
    // a message arriving the instant registration completes can always find
    // its way home.
    {
      const std::lock_guard<std::mutex> lock(registry_mutex());
      registry()[handle] = this;
    }

    if (SickScanApiRegisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud) !=
        SICK_SCAN_API_SUCCESS) {
      unregister_self();
      SickScanApiRelease(handle);
      handle = nullptr;
      throw std::runtime_error("SickScanSource: SickScanApiRegisterPolarPointCloudMsg failed");
    }
    callback_registered = true;

    // Launch file first, then `tag:=value` overrides -- the argument shape
    // the vendor's own test scripts use, verified working against the real
    // picoScan150:
    //   sick_scan_xd_api_test ../launch/sick_picoscan.launch
    //       hostname:=192.168.12.222 udp_receiver_ip:=192.168.12.240
    const std::string launch_args = cfg.launch_file +                             //
                                    " hostname:=" + cfg.hostname +                //
                                    " udp_receiver_ip:=" + cfg.udp_receiver_ip +  //
                                    " custom_pointclouds:=" + cfg.cloud_name;

    const std::int32_t rc = SickScanApiInitByLaunchfile(handle, launch_args.c_str());
    if (rc != SICK_SCAN_API_SUCCESS) {
      SickScanApiDeregisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud);
      callback_registered = false;
      unregister_self();
      SickScanApiRelease(handle);
      handle = nullptr;
      throw std::runtime_error("SickScanSource: SickScanApiInitByLaunchfile failed (rc=" +
                               std::to_string(rc) + ") for args: " + launch_args);
    }
  }

  ~Impl() {
    // Order matters and is the reverse of construction: stop the callbacks
    // first, then take ourselves out of the registry, and only then tear
    // down the handle. Deregistering first guarantees no driver thread is
    // still calling into an object whose members are about to be destroyed.
    if (handle != nullptr) {
      if (callback_registered) {
        SickScanApiDeregisterPolarPointCloudMsg(handle, &Impl::on_polar_cloud);
      }
      unregister_self();
      SickScanApiClose(handle);
      SickScanApiRelease(handle);
    }
  }

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  void unregister_self() {
    const std::lock_guard<std::mutex> lock(registry_mutex());
    registry().erase(handle);
  }

  /// The C entry point the driver calls on its own thread, once per polar
  /// point cloud. Everything useful has to be copied out here and now: the
  /// library frees `msg` the moment this returns (api_impl.cpp,
  /// polar_pointcloud_callback -> freePointCloudMsg).
  static void on_polar_cloud(SickScanApiHandle api_handle, const SickScanPointCloudMsg* msg) {
    if (msg == nullptr) return;
    Impl* self = nullptr;
    {
      const std::lock_guard<std::mutex> lock(registry_mutex());
      const auto it = registry().find(api_handle);
      if (it == registry().end()) return;  // already torn down; drop the message
      self = it->second;
    }
    self->enqueue(*msg);
  }

  void enqueue(const SickScanPointCloudMsg& msg) {
    Scan scan;
    if (!to_scan(msg, scan)) return;

    const std::lock_guard<std::mutex> lock(queue_mutex);
    // Bounded, newest-wins. A caller polling slower than the sensor spins
    // loses the oldest scans rather than growing the queue forever -- for
    // live sensor data, staleness is worse than loss.
    while (queue.size() >= cfg.queue_capacity) {
      queue.pop_front();
      ++dropped;
    }
    queue.push_back(std::move(scan));
    queue_cv.notify_one();
  }

  /// Turns one vendor point cloud into one lidar::Scan. Returns false if the
  /// cloud isn't one we can read (no range/azimuth fields).
  bool to_scan(const SickScanPointCloudMsg& msg, Scan& out) {
    const int offset_range = find_field_offset(msg.fields, "range");
    const int offset_azimuth = find_field_offset(msg.fields, "azimuth");
    const int offset_intensity = find_intensity_offset(msg.fields);
    const int offset_echo = find_field_offset(msg.fields, "echo");

    // Note this is a lookup BY NAME, in any order. The vendor's polling
    // function (SickScanApiWaitNextPolarPointCloudMsg) instead demands
    // fields[0]=="range", fields[1]=="azimuth", fields[2]=="elevation" by
    // position, which silently rejects every cloud that also carries
    // intensity -- since the driver emits "i" before "range". That
    // positional check is a large part of why this class uses the callback
    // API instead.
    if (offset_range < 0 || offset_azimuth < 0) return false;
    if (msg.data.buffer == nullptr || msg.point_step == 0) return false;

    const std::uint64_t num_points =
        static_cast<std::uint64_t>(msg.width) * static_cast<std::uint64_t>(msg.height);
    if (num_points * msg.point_step > msg.data.size) return false;  // truncated buffer

    out.ranges.clear();
    out.intensities.clear();
    out.ranges.reserve(num_points);
    if (offset_intensity >= 0) out.intensities.reserve(num_points);

    float min_azimuth = 0.0F;
    float max_azimuth = 0.0F;
    bool have_point = false;

    for (std::uint64_t i = 0; i < num_points; ++i) {
      const std::uint8_t* point = msg.data.buffer + i * msg.point_step;

      // Single-echo only, matching the "single-echo first" step in the build
      // order. echo is INT8, one byte.
      if (offset_echo >= 0 && read_field<std::int8_t>(point, offset_echo) != 0) {
        continue;
      }

      const float range = read_field<float>(point, offset_range);
      const float azimuth = read_field<float>(point, offset_azimuth);
      out.ranges.push_back(range);
      if (offset_intensity >= 0) {
        out.intensities.push_back(read_field<float>(point, offset_intensity));
      }
      if (!have_point) {
        min_azimuth = max_azimuth = azimuth;
        have_point = true;
      } else {
        min_azimuth = std::min(min_azimuth, azimuth);
        max_azimuth = std::max(max_azimuth, azimuth);
      }
    }

    const std::int64_t stamp_ns =
        static_cast<std::int64_t>(msg.header.timestamp_sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(msg.header.timestamp_nsec);

    // EVERY field is assigned. Anything left unassigned would keep a stale
    // value from whatever Scan this object was reused from.
    out.frame_id = cfg.frame_id;
    out.stamp_ns = stamp_ns;
    // Atomic exchange rather than a plain read-then-write: the vendor
    // delivers on its own thread(s), and nothing in its API promises there
    // is only ever one. This costs nothing and removes the assumption.
    const std::int64_t previous = last_stamp_ns.exchange(stamp_ns);
    const std::int64_t delta_ns = stamp_ns - previous;

    // scan_time_ns means "how long this revolution took", so only a
    // plausible positive interval is reported; anything else yields 0, the
    // same "not measurable" answer the very first scan gives.
    //
    // This is not theoretical. Against the real picoScan150 the driver's
    // timestamp base switches part-way through a run -- the first scans
    // carry the device's uptime clock (~5378 s) and then it jumps to epoch
    // time (~1.79e18 ns) once clock sync settles. A plain subtraction across
    // that boundary reported a single scan as taking 20 years, which then
    // propagated into time_increment_ns. A backwards jump is guarded for the
    // same reason.
    out.scan_time_ns =
        (previous > 0 && delta_ns > 0 && delta_ns <= kMaxPlausibleScanTimeNs) ? delta_ns : 0;
    out.time_increment_ns =
        out.ranges.empty() ? 0
                           : out.scan_time_ns / static_cast<std::int64_t>(out.ranges.size());

    // Derived from this scan's actual azimuths rather than assumed: the
    // picoScan150 reports a per-point azimuth and does not promise uniform
    // spacing, so hardcoding an increment would paper over a real
    // discrepancy.
    out.angle_min = min_azimuth;
    out.angle_max = max_azimuth;
    out.angle_increment = out.ranges.size() > 1
                              ? (max_azimuth - min_azimuth) /
                                    static_cast<float>(out.ranges.size() - 1)
                              : 0.0F;

    out.range_min = cfg.range_min;
    out.range_max = cfg.range_max;
    out.echo_index = 0;
    out.echo_count = msg.num_echos > 0 ? static_cast<std::uint8_t>(msg.num_echos) : 1;
    return true;
  }

  Config cfg;
  SickScanApiHandle handle = nullptr;
  bool callback_registered = false;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<Scan> queue;
  std::uint64_t dropped = 0;

  // Written from the driver thread inside to_scan(); atomic so that two
  // concurrent deliveries cannot tear it.
  std::atomic<std::int64_t> last_stamp_ns{0};
};

SickScanSource::SickScanSource(Config cfg) : impl_(std::make_unique<Impl>(std::move(cfg))) {}
SickScanSource::~SickScanSource() = default;
SickScanSource::SickScanSource(SickScanSource&&) noexcept = default;
SickScanSource& SickScanSource::operator=(SickScanSource&&) noexcept = default;

bool SickScanSource::poll_scan(Scan& out, int timeout_ms) noexcept {
  try {
    std::unique_lock<std::mutex> lock(impl_->queue_mutex);
    if (!impl_->queue_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                  [this] { return !impl_->queue.empty(); })) {
      return false;  // nothing arrived in time -- the ordinary case, not an error
    }
    out = std::move(impl_->queue.front());
    impl_->queue.pop_front();
    return true;
  } catch (...) {
    // poll_scan is noexcept; a throw here would call std::terminate. Locking
    // a non-recursive mutex realistically cannot fail, but "realistically"
    // is not a guarantee worth crashing a vehicle over.
    return false;
  }
}

}  // namespace lidar
