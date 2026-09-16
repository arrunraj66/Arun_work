// Transport round trip, over inproc  one process, two sockets, no network.

#include "mw/transport.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  std::printf("%-58s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

// A PUB socket throws away anything sent before a subscriber has finished
// connecting. This is the ZeroMQ "slow joiner" problem, and it is not a bug 
// there is genuinely nobody to deliver to yet. Retry until one gets through.
std::optional<mw::Message> publish_until_received(mw::Publisher& pub, mw::Subscriber& sub,
                                                  std::string_view topic,
                                                  std::string_view payload) {
  for (int attempt = 0; attempt < 200; ++attempt) {
    (void)pub.publish(topic, payload);
    if (auto msg = sub.receive(10)) return msg;
  }
  return std::nullopt;
}

}  // namespace

int main() {
  mw::Context ctx;

  const std::string endpoint = "inproc://mw_test";

  mw::Publisher  pub(ctx, endpoint);
  mw::Subscriber sub(ctx, endpoint);
  sub.subscribe("nav.");

  // 1. a matching topic arrives, both frames intact
  {
    auto got = publish_until_received(pub, sub, "nav.pose", std::string("\x01\x02\x00\x03", 4));
    check(got.has_value(), "matching topic is delivered");
    if (got) {
      check(got->topic == "nav.pose", "topic frame survives the round trip");
      check(got->payload.size() == 4, "payload keeps its length (embedded NUL)");
      check(got->payload[2] == '\0', "payload is binary-clean");
    }
  }

  // 2. a non-matching topic is filtered out
  {
    for (int i = 0; i < 20; ++i) (void)pub.publish("sonar.ping", "xxxx");
    auto got = sub.receive(50);
    check(!got.has_value(), "non-matching topic is filtered out");
  }

  // 3. the prefix is a prefix, not an exact name
  {
    auto got = publish_until_received(pub, sub, "nav.depth", "d");
    check(got.has_value() && got->topic == "nav.depth", "subscribe() matches by prefix");
  }

  // 4. a timeout with nothing to read is not an error
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto got = sub.receive(60);
    const auto dt = std::chrono::steady_clock::now() - t0;
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
    check(!got.has_value(), "an empty queue returns nothing, not an error");
    check(ms >= 50, "receive() actually waited for its timeout");
  }

  // 5. nothing was dropped on this tiny run
  check(pub.dropped() == 0, "no drops on an unloaded publisher");

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
              failures, failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}