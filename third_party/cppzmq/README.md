# third_party

Code we did not write, vendored deliberately and pinned to a version.

Nothing in here is edited. If a fix is needed, it is fetched again at a newer
tag and this file records the change  never a local patch, because a local
patch is invisible the next time somebody updates.

## cppzmq

| | |
|---|---|
| file | `cppzmq/zmq.hpp` |
| version | v4.8.1 |
| upstream | https://github.com/zeromq/cppzmq |
| licence | MIT |
| fetched | 2026-09-16 |

Header-only C++ wrapper over libzmq's C API. It compiles into our objects and
nothing links against it, which is why one file is the whole of it.

Pinned at v4.8.1 to match libzmq 4.3.x, the version Ubuntu 22.04 ships.
`libzmq3-dev` is still a system package  only the C++ header is vendored.

To refresh:

    curl -L -o third_party/cppzmq/zmq.hpp \
      https://raw.githubusercontent.com/zeromq/cppzmq/v4.8.1/zmq.hpp