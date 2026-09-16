// Shows what the bytes actually look like. Run it once and the message header
// stops being an abstraction.

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>

#include "mw/heartbeat.pb.h"
#include "mw/message.hpp"

namespace {

void hexdump(const std::string& bytes) {
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i % 16 == 0) std::printf("\n  %04zu  ", i);
    std::printf("%02x ", static_cast<unsigned char>(bytes[i]));
  }
  std::printf("\n");
}

}  // namespace

int main() {
  std::uint64_t sequence = 0;

  mw::proto::Heartbeat hb;
  mw::stamp(*hb.mutable_header(), "auv-control", sequence);
  hb.set_node_name("auv-control");
  hb.set_state(mw::proto::NODE_STATE_RUNNING);
  hb.set_tick_count(42);
  hb.set_wcet_us(910);

  std::string bytes;
  hb.SerializeToString(&bytes);

  std::cout << "Heartbeat, as text:\n" << hb.DebugString();
  std::cout << "serialised size: " << bytes.size() << " bytes";
  hexdump(bytes);

  mw::proto::Header peeked;
  const mw::HeaderStatus status = mw::peek_header(bytes, peeked);
  std::cout << "\npeek_header without knowing the payload type: "
            << mw::to_string(status) << '\n'
            << "  publisher       " << peeked.publisher() << '\n'
            << "  sequence        " << peeked.sequence() << '\n'
            << "  publish_time_ns " << peeked.publish_time_ns() << '\n';
  return 0;
}
