find_path(SICK_SCAN_XD_INCLUDE_DIR
  NAMES sick_scan_xd_api/sick_scan_api.h
  HINTS ${SICK_SCAN_XD_DIR}/include
)

find_library(SICK_SCAN_XD_LIBRARY
  NAMES sick_scan_xd_shared_lib
  HINTS ${SICK_SCAN_XD_DIR}/build ${SICK_SCAN_XD_DIR}/lib
)

if(SICK_SCAN_XD_INCLUDE_DIR AND SICK_SCAN_XD_LIBRARY)
  set(SICK_SCAN_XD_FOUND TRUE)
  message(STATUS "  sick_scan_xd : ${SICK_SCAN_XD_LIBRARY}")
else()
  set(SICK_SCAN_XD_FOUND FALSE)
  message(STATUS "  sick_scan_xd : not found -- lidar_sick target skipped "
                 "(set -DSICK_SCAN_XD_DIR=/path/to/sick_scan_xd once it's built)")
endif()