// mw_pub  publish heartbeats until you stop it.
//
//   ./mw_pub  [endpoint]  [topic]  [period_ms]
//
// Defaults to ipc:///tmp/mw_demo, topic health.heartbeat, 10 Hz.

#include "mw/message.hpp"
#include "mw/time.hpp"
#include "mw/transport.hpp"
#include "mw/heartbeat.pb.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

int main(int argc, char** argv) {
  const std::string endpoint = argc > 1 ? argv[1] : "ipc:///tmp/mw_demo";
  const std::string topic    = argc > 2 ? argv[2] : "health.heartbeat";
  const int period_ms        = argc > 3 ? std::atoi(argv[3]) : 100;

  mw::Context   ctx;
  mw::Publisher pub(ctx, endpoint);

  std::printf("publishing %s on %s every %d ms  ctrl-c to stop\n",
              topic.c_str(), endpoint.c_str(), period_ms);

  std::uint64_t sequence = 0;
  const std::int64_t t0 = mw::steady_time_ns();

  for (;;) {
    mw::proto::Heartbeat hb;
    mw::stamp(*hb.mutable_header(), "auv-control", sequence);
    hb.set_node_name("auv-control");
    hb.set_state(mw::proto::NODE_STATE_RUNNING);
    hb.set_tick_count(sequence);
    hb.set_overruns(0);
    hb.set_wcet_us(static_cast<std::uint32_t>(
        (mw::steady_time_ns() - t0) % 1000));

    const std::string bytes = hb.SerializeAsString();
    const bool ok = pub.publish(topic, bytes);

    std::printf("\rseq %-8llu  %2zu bytes  %s  dropped %llu",
                static_cast<unsigned long long>(sequence), bytes.size(),
                ok ? "sent" : "DROP", static_cast<unsigned long long>(pub.dropped()));
    std::fflush(stdout);

    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}