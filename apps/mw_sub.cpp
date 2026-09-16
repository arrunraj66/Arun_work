// mw_sub  print every message whose topic starts with a given prefix.
//
//   ./mw_sub  [endpoint]  [prefix]
//
// Defaults to ipc:///tmp/mw_demo and an empty prefix, which means everything.

#include "mw/message.hpp"
#include "mw/transport.hpp"
#include "mw/heartbeat.pb.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv) {
  const std::string endpoint = argc > 1 ? argv[1] : "ipc:///tmp/mw_demo";
  const std::string prefix   = argc > 2 ? argv[2] : "";

  mw::Context    ctx;
  mw::Subscriber sub(ctx, endpoint);
  sub.subscribe(prefix);

  std::printf("subscribed to \"%s\" on %s  ctrl-c to stop\n\n",
              prefix.c_str(), endpoint.c_str());

  std::uint64_t expected = 0;
  bool          first    = true;

  for (;;) {
    auto msg = sub.receive(1000);
    if (!msg) { std::printf("  (idle)\n"); continue; }

    // Step 1: read only the header, without knowing the message type.
    mw::proto::Header header;
    const auto status = mw::peek_header(msg->payload, header);
    if (status != mw::HeaderStatus::kOk) {
      std::printf("  %-20s REJECTED: %s\n", msg->topic.c_str(),
                  std::string(mw::to_string(status)).c_str());
      continue;
    }

    // Step 2: only now decide what to do with the body.
    mw::proto::Heartbeat hb;
    const bool parsed = hb.ParseFromString(msg->payload);

    const std::uint64_t seq = header.sequence();
    const char* gap = "";
    if (!first && seq != expected) gap = "  <-- GAP";
    expected = seq + 1;
    first    = false;

    std::printf("  %-20s seq %-6llu from %-12s state %d%s\n",
                msg->topic.c_str(), static_cast<unsigned long long>(seq),
                header.publisher().c_str(),
                parsed ? static_cast<int>(hb.state()) : -1, gap);
  }
}