// Steps 4+5 together: a Scan published on one socket comes out of a
// subscriber on another, field for field. They can only be tested as a
// pair, so they are tested as a pair.
//
// Uses ONLY the public lidar:: API -- no ZeroMQ, no protobuf, no mw
// headers. That is the point being proved as much as the round trip is:
// a consumer of this library sends and receives Scans without ever
// learning what carries them.

#include "lidar/publisher.hpp"
#include "lidar/scan.hpp"
#include "lidar/source.hpp"
#include "lidar/subscriber.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-64s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

lidar::Scan make_scan(int i) {
  lidar::Scan scan;
  scan.frame_id = "test_pubsub";
  scan.stamp_ns = 1'700'000'000'000'000'000LL + static_cast<std::int64_t>(i) * 40'000'000LL;
  scan.scan_time_ns = 40'000'000;
  scan.time_increment_ns = 40'000'000 / 900;
  scan.angle_min = -2.409F;
  scan.angle_max = 2.409F;
  scan.angle_increment = 0.00504F;
  scan.range_min = 0.05F;
  scan.range_max = 120.0F;
  scan.echo_index = 0;
  scan.echo_count = 1;
  scan.ranges = {1.0F + static_cast<float>(i), 2.5F, 3.25F, 4.125F};
  scan.intensities = {10.0F, 20.0F, 30.0F, 40.0F};
  return scan;
}

bool same_scan(const lidar::Scan& a, const lidar::Scan& b) {
  return a.frame_id == b.frame_id && a.stamp_ns == b.stamp_ns &&
         a.scan_time_ns == b.scan_time_ns && a.time_increment_ns == b.time_increment_ns &&
         a.angle_min == b.angle_min && a.angle_max == b.angle_max &&
         a.angle_increment == b.angle_increment && a.range_min == b.range_min &&
         a.range_max == b.range_max && a.echo_index == b.echo_index &&
         a.echo_count == b.echo_count && a.ranges == b.ranges && a.intensities == b.intensities;
}

}  // namespace

int main() {
  // ipc:// rather than tcp://: no port to collide with another developer,
  // another test, or whatever else is running on the build machine.
  const std::string endpoint = "ipc:///tmp/lidar_test_pubsub.ipc";

  lidar::ScanPublisher publisher(endpoint);
  lidar::ScanSubscriber subscriber(endpoint);

  check(publisher.topic() == "lidar.scan" && subscriber.topic() == "lidar.scan",
        "publisher and subscriber agree on the default topic");

  // PUB/SUB drops everything sent before the subscriber has finished
  // connecting -- the "slow joiner" problem. Rather than sleeping and
  // hoping, publish the same scan until one arrives. This is what any real
  // caller does anyway: a 25 Hz sensor sends another one in 40 ms.
  const lidar::Scan sent = make_scan(0);
  lidar::Scan received;
  bool got_one = false;
  for (int attempt = 0; attempt < 200 && !got_one; ++attempt) {
    (void)publisher.publish(sent);
    got_one = subscriber.poll_scan(received, /*timeout_ms=*/20);
  }
  check(got_one, "a published scan reaches the subscriber");
  check(got_one && same_scan(sent, received), "every field survives the round trip");

  // Now that the connection is up, a burst should arrive in order.
  constexpr int kBurst = 10;
  for (int i = 1; i <= kBurst; ++i) {
    (void)publisher.publish(make_scan(i));
  }
  int in_order = 0;
  for (int i = 1; i <= kBurst; ++i) {
    lidar::Scan s;
    if (!subscriber.poll_scan(s, 500)) break;
    if (!same_scan(make_scan(i), s)) break;
    ++in_order;
  }
  check(in_order == kBurst, "a burst of 10 scans arrives complete and in order");

  check(subscriber.malformed() == 0, "nothing arrived that failed to parse");

  // Timeout must be reported as "nothing", not as a stale scan.
  lidar::Scan unused;
  check(!subscriber.poll_scan(unused, 50), "poll_scan times out cleanly when nothing is sent");

  // The reason ScanSubscriber derives from IScanSource: anything written
  // against the interface works over the network without knowing it.
  {
    std::unique_ptr<lidar::IScanSource> as_source =
        std::make_unique<lidar::ScanSubscriber>(endpoint);
    lidar::Scan via_interface;
    bool got = false;
    for (int attempt = 0; attempt < 200 && !got; ++attempt) {
      (void)publisher.publish(sent);
      got = as_source->poll_scan(via_interface, 20);
    }
    check(got, "a ScanSubscriber used through IScanSource* delivers scans");
    check(got && same_scan(sent, via_interface),
          "scans received through the interface are identical");
  }

  // A subscriber on a different topic must hear nothing.
  {
    lidar::ScanSubscriber other(endpoint, "some.other.topic");
    check(other.topic() == "some.other.topic", "subscriber honours a non-default topic");
    for (int i = 0; i < 20; ++i) (void)publisher.publish(sent);
    lidar::Scan nothing;
    check(!other.poll_scan(nothing, 100), "a subscriber on another topic receives nothing");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
