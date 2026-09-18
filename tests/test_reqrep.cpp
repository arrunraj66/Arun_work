// mw::Requester / mw::Replier: the REQ/REP pair underneath the query
// service, tested on their own before anything is built on top of them.
//
// The property that matters most is what happens when nobody answers -- a
// plain REQ socket has no real timeout and gets permanently stuck if you do
// not handle it. That is test 2.

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
  std::printf("%-62s %s\n", what, ok ? "ok" : "FAIL");
  if (!ok) ++failures;
}

}  // namespace

int main() {
  // 1. The real shape: a server on its own thread, a client on the main one.
  {
    const std::string endpoint = "ipc:///tmp/mw_test_reqrep.ipc";
    mw::Context ctx_client, ctx_server;
    mw::Requester req(ctx_client, endpoint);
    mw::Replier   rep(ctx_server, endpoint);

    bool server_done = false;
    std::thread server([&] {
      for (int i = 0; i < 3; ++i) {
        rep.serve_one(2000, [](const std::string& r) { return "echo:" + r; });
      }
      server_done = true;
    });

    check(req.request("one", 2000) == std::make_optional<std::string>("echo:one"),
          "a served request returns the handler's exact reply");
    check(req.request("two", 2000) == std::make_optional<std::string>("echo:two"),
          "a second request on the same Requester works after the first");
    check(req.request("three", 2000) == std::make_optional<std::string>("echo:three"),
          "and a third");

    server.join();
    check(server_done, "the server thread served all three and returned");
  }

  // 2. No server at all: request() must time out and come back clean,
  //    rather than hanging or leaving the Requester stuck for next time.
  {
    const std::string endpoint = "ipc:///tmp/mw_test_reqrep_nobody.ipc";
    mw::Context ctx;
    mw::Requester req(ctx, endpoint);

    const auto start = std::chrono::steady_clock::now();
    const std::optional<std::string> reply = req.request("anyone home?", 200);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    check(!reply.has_value(), "a request with nobody listening times out rather than hanging");
    check(elapsed < std::chrono::milliseconds(2000),
          "and it returns promptly -- close to the timeout, not much longer");

    // The real point: the Requester must still be usable after a timeout. A
    // raw REQ socket would refuse this -- it is still "owed" the reply to
    // the request that never came. Start a real server on the same endpoint
    // now and prove the SAME Requester object can still talk to it.
    mw::Context ctx_server;
    mw::Replier rep(ctx_server, endpoint);
    std::thread server([&] { rep.serve_one(3000, [](const std::string&) { return "alive"; }); });

    std::optional<std::string> second;
    for (int attempt = 0; attempt < 50 && !second.has_value(); ++attempt) {
      second = req.request("still there?", 100);
    }
    server.join();
    check(second == std::make_optional<std::string>("alive"),
          "and the SAME Requester recovers and works once a server appears");
  }

  // 3. serve_one() returns false, not a crash, when nothing arrives.
  {
    mw::Context ctx;
    mw::Replier rep(ctx, "ipc:///tmp/mw_test_reqrep_idle.ipc");
    const bool served = rep.serve_one(50, [](const std::string& r) { return r; });
    check(!served, "serve_one() returns false on a timeout with no request");
  }

  std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
              failures == 1 ? "" : "s");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}