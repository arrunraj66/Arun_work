// nav_avoid_main.cpp -- Navigation Step 1's runnable half: subscribes to
// three of the six Webots lidar topics (front/left/right -- this step only
// handles forward motion; rear/top/bottom are unused for now, see the
// README note this ships with), decides a steering command with
// nav::decide(), and publishes it for the Webots controller to apply.
//
// This is the first process in the repo that is BOTH a subscriber and a
// publisher at once -- a real "node" in the sense the middleware was built
// around, not a new capability the middleware needed. Sensing (the six
// lidars), deciding (this process), and acting (the Webots controller) are
// three separate participants in one pub/sub graph, exactly the pattern
// the multi-sensor plan doc lays out.
//
// Safety default worth being explicit about: a sector with no data yet
// (subscriber hasn't received a scan since this process started, or the
// controller/lidar isn't running) is treated as range 0.0 -- "obstacle
// right here" -- not as "clear, go ahead." Missing data must never look
// like a clear path. See main()'s SectorRanges initialisation below.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "lidar/scan.hpp"
#include "lidar/subscriber.hpp"
#include "nav/avoider.hpp"
#include "nav/nav_publisher.hpp"

namespace {

// The closest VALID reading in a scan, or +infinity if the scan has no
// valid points at all (sensor saw nothing in range) -- distinct from "no
// scan received yet," which the caller tracks separately and treats as 0,
// not infinity. See the file header for why those two cases must not be
// conflated.
[[nodiscard]] float nearest_valid_range(const lidar::Scan& scan) noexcept {
  float nearest = std::numeric_limits<float>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    if (lidar::is_valid(scan, i) && scan.ranges[i] < nearest) {
      nearest = scan.ranges[i];
    }
  }
  return nearest;
}

}  // namespace

int main(int argc, char** argv) {
  // Usage: nav_avoid_main [lidar_endpoint] [nav_endpoint]
  //   lidar_endpoint: where the Webots controller's six-lidar publisher is
  //                   (default matches its own tcp://*:5656 bind)
  //   nav_endpoint:   where THIS process binds its own nav.cmd publisher
  const std::string lidar_endpoint = (argc > 1) ? argv[1] : "tcp://127.0.0.1:5656";
  const std::string nav_endpoint = (argc > 2) ? argv[2] : "tcp://*:5657";

  std::cout << "nav_avoid_main: subscribing to " << lidar_endpoint
            << " (topics sensor.lidar.{front,left,right}.scan)" << std::endl;
  std::cout << "nav_avoid_main: publishing nav.cmd on " << nav_endpoint << std::endl;

  lidar::ScanSubscriber front_sub(lidar_endpoint, "sensor.lidar.front.scan");
  lidar::ScanSubscriber left_sub(lidar_endpoint, "sensor.lidar.left.scan");
  lidar::ScanSubscriber right_sub(lidar_endpoint, "sensor.lidar.right.scan");

  nav::NavPublisher publisher(nav_endpoint, "nav.cmd");
  const nav::AvoiderConfig cfg{};  // defaults: see avoider.hpp

  // 0.0, not infinity -- "no scan received yet" fails safe (tier 3: stop
  // and turn), never fails open. Overwritten the first time each sector
  // actually receives a scan.
  nav::SectorRanges sectors{0.0F, 0.0F, 0.0F};

  std::uint64_t ticks = 0;
  std::uint64_t published = 0;
  std::uint64_t dropped = 0;

  // Control loop rate: 20 Hz matches the lidar publish rate we've measured
  // in practice, so there's a fresh scan to react to almost every tick
  // without polling far faster than new data can arrive.
  constexpr auto kTickPeriod = std::chrono::milliseconds(50);
  constexpr int kPollTimeoutMs = 5;  // short: don't let one quiet sensor stall the tick

  while (true) {
    lidar::Scan scan;

    if (front_sub.poll_scan(scan, kPollTimeoutMs)) {
      sectors.front = nearest_valid_range(scan);
    }
    if (left_sub.poll_scan(scan, kPollTimeoutMs)) {
      sectors.left = nearest_valid_range(scan);
    }
    if (right_sub.poll_scan(scan, kPollTimeoutMs)) {
      sectors.right = nearest_valid_range(scan);
    }

    nav::NavCommand cmd = nav::decide(sectors, cfg);
    cmd.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();

    if (publisher.publish(cmd)) {
      ++published;
    } else {
      ++dropped;
    }

    ++ticks;
    if (ticks % 100 == 0) {
      std::cout << "nav_avoid_main: tick " << ticks << " | front=" << sectors.front
                << " left=" << sectors.left << " right=" << sectors.right
                << " | forward_speed=" << cmd.forward_speed << " turn_rate=" << cmd.turn_rate
                << " obstacle=" << (cmd.obstacle_detected ? "yes" : "no")
                << " | published=" << published << " dropped=" << dropped << std::endl;
    }

    std::this_thread::sleep_for(kTickPeriod);
  }

  return 0;
}
