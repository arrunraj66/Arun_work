# Optional: SickScanSource only builds if sick_scan_xd's real API is present
# on this machine. This find is deliberately soft (no REQUIRED) because
# almost everything else in this project -- Steps 1-8, the WebSocket bridge,
# every existing test -- never touches real hardware or the vendor SDK at
# all, and must keep building on a machine that has neither.
#
# Point -DSICK_SCAN_XD_DIR=/path/to/sick_scan_xd at your built checkout if
# CMake doesn't find it automatically (e.g. it's not installed system-wide).
#
# The second HINTS entry below (~/sick_scan_ws/sick_scan_xd) is this specific
# team's actual build location -- confirmed present on the dev workstation.
# It is a FALLBACK, not a requirement: -DSICK_SCAN_XD_DIR always wins when
# given, so a second checkout elsewhere (a different machine, a future CI
# box) is never blocked by this default. The point of adding it is that a
# bare `cmake -S . -B build`, with no flags at all, now finds the SDK on
# this machine without anyone having to remember or retype the -D flag --
# which matters because `rm -rf build` silently forgets it every time.

find_path(SICK_SCAN_XD_INCLUDE_DIR
  NAMES sick_scan_xd_api/sick_scan_api.h
  HINTS ${SICK_SCAN_XD_DIR}/include
        $ENV{HOME}/sick_scan_ws/sick_scan_xd/include
)

find_library(SICK_SCAN_XD_LIBRARY
  NAMES sick_scan_xd_shared_lib
  HINTS ${SICK_SCAN_XD_DIR}/build ${SICK_SCAN_XD_DIR}/lib
        $ENV{HOME}/sick_scan_ws/sick_scan_xd/build
)

if(SICK_SCAN_XD_INCLUDE_DIR AND SICK_SCAN_XD_LIBRARY)
  set(SICK_SCAN_XD_FOUND TRUE)
  message(STATUS "  sick_scan_xd : ${SICK_SCAN_XD_LIBRARY}")
else()
  set(SICK_SCAN_XD_FOUND FALSE)
  message(STATUS "  sick_scan_xd : not found -- lidar_sick target skipped "
                 "(set -DSICK_SCAN_XD_DIR=/path/to/sick_scan_xd once it's built)")
endif()