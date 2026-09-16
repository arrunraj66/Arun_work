# libzmq ships a pkg-config file on every platform we care about, so that is how
# we find the library itself. cppzmq is a different matter: it is a single
# header, and not every distribution packages it. So we look in two places and
# say clearly what to do if neither has it.
find_package(PkgConfig REQUIRED)
pkg_check_modules(libzmq REQUIRED IMPORTED_TARGET libzmq)

# An INTERFACE target so the rest of the tree says `mw_zeromq` and never has to
# know how it was found.
add_library(mw_zeromq INTERFACE)
target_link_libraries(mw_zeromq INTERFACE PkgConfig::libzmq)

# ---- cppzmq: a vendored copy wins, an installed one is the fallback ----------
set(MW_CPPZMQ_VENDORED "${PROJECT_SOURCE_DIR}/third_party/cppzmq")

if(EXISTS "${MW_CPPZMQ_VENDORED}/zmq.hpp")
  set(MW_CPPZMQ_INCLUDE_DIR "${MW_CPPZMQ_VENDORED}")
  set(MW_CPPZMQ_ORIGIN      "vendored")
else()
  find_path(MW_CPPZMQ_INCLUDE_DIR zmq.hpp HINTS ${libzmq_INCLUDE_DIRS})
  set(MW_CPPZMQ_ORIGIN "${MW_CPPZMQ_INCLUDE_DIR}")
endif()

if(NOT MW_CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR
    "zmq.hpp (cppzmq) not found.\n"
    "  Either:  sudo apt install cppzmq-dev\n"
    "  Or vendor it (no package needed):\n"
    "    mkdir -p ${MW_CPPZMQ_VENDORED}\n"
    "    curl -L -o ${MW_CPPZMQ_VENDORED}/zmq.hpp \\\n"
    "      https://raw.githubusercontent.com/zeromq/cppzmq/v4.8.1/zmq.hpp")
endif()

# SYSTEM: zmq.hpp is not our code, and we do not want our own -Wold-style-cast
# and -Wconversion turning someone else's header into build errors.
target_include_directories(mw_zeromq SYSTEM INTERFACE "${MW_CPPZMQ_INCLUDE_DIR}")

message(STATUS "  zeromq     : ${libzmq_VERSION}")
message(STATUS "  cppzmq     : ${MW_CPPZMQ_ORIGIN}")