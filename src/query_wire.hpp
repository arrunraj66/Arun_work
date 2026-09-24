#pragma once
//
// src/query_wire.hpp — the private wire format the query client and server
// agree on. Never included by include/lidar/, same rule as scan_codec.hpp:
// nothing that links this library needs to know protobuf is involved.
//
// The response to a query is zero or more scans:
//
//   [4 bytes: count, little-endian]
//   for each scan:
//     [4 bytes: length of this scan's protobuf bytes, little-endian]
//     [that many bytes: a serialised proto::LidarScan]
//
// This is the exact same length-prefix convention ScanRecorder's log file
// uses (see write_length_prefix() there) -- little-endian, spelled out byte
// by byte, because the file's rule ("must mean the same thing everywhere")
// applies just as much to bytes crossing a socket as to bytes on disk. It is
// reimplemented here rather than shared, because the log writes to an
// std::ofstream and this writes into an in-memory std::string -- different
// enough plumbing that sharing the function would mean an awkward stream
// abstraction for one four-line helper.

#include <cstdint>
#include <string>
#include <vector>

#include "lidar/scan.hpp"

namespace lidar {

/// Every scan the server is replying with, framed as above.
[[nodiscard]] std::string encode_scans(const std::vector<Scan>& scans);

/// The inverse. Throws std::runtime_error if `bytes` is not validly framed
/// -- which only happens if the two ends of the wire disagree about the
/// protocol, not as part of ordinary "no scans" traffic (that is an
/// honestly-encoded count of 0, not malformed bytes).
[[nodiscard]] std::vector<Scan> decode_scans(const std::string& bytes);

}  // namespace lidar
