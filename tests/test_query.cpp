// ScanQueryClient / ScanQueryServer: the pull-based backup path to
// ScanSubscriber, proven end to end against a real recording.
//
// Uses ONLY the public lidar:: API -- ScanRecorder to seed data,
// ScanQueryServer, ScanQueryClient. No src/ include dir, no protobuf, no
// ZeroMQ headers, same discipline as test_pubsub.

#include "lidar/query_client.hpp"
#include "lidar/query_server.hpp"
#include "lidar/recorder.hpp"
#include "lidar/scan.hpp"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-62s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

lidar::Scan make_scan(int i) {
  lidar::Scan s;
  s.frame_id = "test_query";
  s.stamp_ns = 1'700'000'000'000'000'000LL + static_cast<std::int64_t>(i) * 40'000'000LL;
  s.scan_time_ns = 40'000'000;
  s.angle_min = -2.409F;
  s.angle_max = 2.409F;
  s.angle_increment = 0.00493F;
  s.range_min = 0.05F;
  s.range_max = 120.0F;
  s.echo_count = 1;
  s.ranges = {1.0F + static_cast<float>(i), 2.5F, 3.25F};
  return s;
}

bool same_scan(const lidar::Scan& a, const lidar::Scan& b) {
  return a.frame_id == b.frame_id && a.stamp_ns == b.stamp_ns && a.ranges == b.ranges;
}

}  // namespace

int main() {
  const std::string log_path = "/tmp/test_query.log";
  const std::string db_path  = "/tmp/test_query.db";
  const std::string endpoint = "ipc:///tmp/test_query.ipc";
  std::remove(log_path.c_str());
  std::remove(db_path.c_str());

  // Seed a recording of 10 scans before any server exists -- the server
  // reads whatever is already on disk, same as ScanReader always has.
  for (int i = 0; i < 10; ++i) {
    lidar::ScanRecorder rec(log_path, db_path);
    rec.record(make_scan(i));
  }

  lidar::ScanQueryServer server(log_path, db_path, endpoint);
  check(server.served() == 0, "a fresh server has answered nothing yet");

  // Serve requests on their own thread; the client blocks on the main one,
  // same shape as test_reqrep.
  bool keep_serving = true;
  std::thread server_thread([&] {
    while (keep_serving) (void)server.serve_one(100);
  });

  lidar::ScanQueryClient client(endpoint, /*timeout_ms=*/2000);

  {
    const std::optional<lidar::Scan> latest = client.latest();
    check(client.ok(), "latest() reaches a running server");
    check(latest.has_value() && same_scan(*latest, make_scan(9)),
          "and returns the most recently recorded scan, not the first");
  }

  {
    const std::vector<lidar::Scan> got =
        client.range(make_scan(2).stamp_ns, make_scan(5).stamp_ns);
    check(client.ok(), "range() reaches a running server");
    check(got.size() == 4, "and returns exactly the scans inside the window");
    bool in_order = got.size() == 4;
    for (std::size_t i = 0; in_order && i < got.size(); ++i) {
      in_order = same_scan(got[i], make_scan(2 + static_cast<int>(i)));
    }
    check(in_order, "in recording order, field for field");
  }

  {
    const std::vector<lidar::Scan> got = client.range(0, 1);   // long before any scan
    check(client.ok(), "a range with no matches still reaches the server");
    check(got.empty(), "and an empty vector correctly means \"nothing in range\"");
  }
  {
    // Scans are 40ms apart (see make_scan). A 120ms window anchored to the
    // newest scan (#9) should reach exactly back to #6: 9,8,7,6 are each
    // within 120ms of #9's own stamp_ns; #5 (160ms back) is not.
    const std::vector<lidar::Scan> got = client.recent(120'000'000);
    check(client.ok(), "recent() reaches a running server");
    check(got.size() == 4, "and returns exactly the scans within the window of the newest one");
    bool in_order = got.size() == 4;
    for (std::size_t i = 0; in_order && i < got.size(); ++i) {
      in_order = same_scan(got[i], make_scan(6 + static_cast<int>(i)));
    }
    check(in_order, "in recording order, field for field");
  }

  {
    // The real point of anchoring to the newest scan's OWN stamp_ns rather
    // than wall-clock "now": these scans are timestamped in 2023, not
    // whenever this test happens to run. A wall-clock-anchored version
    // would find nothing here; asking for a window bigger than the whole
    // recording anchored to the newest scan correctly returns everything.
    const std::vector<lidar::Scan> got = client.recent(1'000'000'000);   // 1s >> 360ms span
    check(client.ok(), "recent() with a window covering the whole recording reaches the server");
    check(got.size() == 10, "and returns every scan, confirming it anchors to the newest "
                             "RECORDED stamp, not wall-clock now");
  }
  keep_serving = false;
  server_thread.join();
  check(server.served() == 5, "the server counted exactly the five requests answered");

  // Now the point of the whole feature: nobody is listening any more.
  {
    lidar::ScanQueryClient orphan(endpoint, /*timeout_ms=*/200);
    const std::optional<lidar::Scan> got = orphan.latest();
    check(!got.has_value(), "with no server running, latest() returns nothing");
    check(!orphan.ok(), "and ok() correctly reports it as unreachable, not \"empty\"");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}