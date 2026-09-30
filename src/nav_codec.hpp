#pragma once
//
// src/nav_codec.hpp -- the ONLY file that mentions both nav::NavCommand and
// nav::proto::NavCommand. Same role scan_codec.hpp/cloud_codec.hpp play for
// their types: lives in src/, so no consumer of nav::nav ever sees protobuf.

#include "nav/command.hpp"
#include "nav_command.pb.h"

namespace nav {

/// NavCommand -> wire format. Overwrites everything already in `out`.
void to_proto(const NavCommand& cmd, proto::NavCommand& out);

/// Wire format -> NavCommand.
[[nodiscard]] NavCommand from_proto(const proto::NavCommand& msg);

}  // namespace nav
