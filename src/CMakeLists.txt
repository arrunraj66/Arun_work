# ── the library itself ──────────────────────────────────────────────────────
add_library(mw
  version.cpp
  time.cpp
  message.cpp
  transport.cpp
  acoustic.cpp
  acoustic_loopback.cpp
  executor.cpp             # <-- stage 3: the node runner
)
add_library(mw::mw ALIAS mw)

# mw gets compiled into liblidar.so (privately). Objects that end up inside
# a shared library have to be position-independent, so this is not optional.
set_target_properties(mw PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mw
  PUBLIC  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(mw
  PRIVATE
    MW_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    MW_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    MW_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    MW_VERSION_STRING="${PROJECT_VERSION}"
)

target_link_libraries(mw
  PRIVATE mw_warnings          # our discipline, not our users'
  PUBLIC  mw_sanitizers        # must match across the whole program
  PUBLIC  mw_proto             # our public headers include the generated ones
  PRIVATE mw_zeromq            # <-- add this
)

add_library(lidar SHARED
  scan_codec.cpp
  cloud_codec.cpp          # <-- 3D: PointCloud <-> LidarPointCloud
  recorder.cpp
  reader.cpp
  publisher.cpp
  subscriber.cpp
  cloud_publisher.cpp      # <-- 3D: CloudPublisher, built on mw::Publisher
  cloud_subscriber.cpp     #     and CloudSubscriber, same as the 2D pair
  query_wire.cpp           # <-- backup path: the query request/response
  query_client.cpp         #     wire format, and the two ends of it
  query_server.cpp
)
add_library(lidar::lidar ALIAS lidar)

# VERSION/SOVERSION give the .so a real name (liblidar.so.0.1.0) with a
# liblidar.so.0 symlink. The SOVERSION is the compatibility promise.
set_target_properties(lidar PROPERTIES
  VERSION   ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(lidar
  PUBLIC  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(lidar PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)

target_link_libraries(lidar
  PRIVATE lidar_proto       # our public headers never include the generated ones
  PRIVATE SQLite::SQLite3   # same reason -- recorder.hpp never mentions sqlite3.h
  PRIVATE mw::mw            # ScanPublisher/ScanSubscriber are built on mw's
)                           # ZeroMQ transport; PRIVATE so lidar's consumers
                            # never need mw/transport.hpp or ZeroMQ themselves.

# SickScanSource -- only builds once sick_scan_xd's real library is present
# (see cmake/SickScanXd.cmake). A separate target from `lidar` on purpose:
# the core library (Scan, IScanSource, the codec) must keep building on any
# machine, with or without the vendor SDK installed. Only whatever process
# actually owns the sensor -- sick_scan_source_demo, for now -- needs to
# link lidar::sick.
if(SICK_SCAN_XD_FOUND)
  add_library(lidar_sick
    sick_scan_source.cpp
    sick_cloud_source.cpp   # <-- 3D: SickCloudSource, reads elevation
  )
  add_library(lidar::sick ALIAS lidar_sick)

  target_include_directories(lidar_sick
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${SICK_SCAN_XD_INCLUDE_DIR}
  )
  target_compile_options(lidar_sick PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(lidar_sick
    PUBLIC  lidar::lidar          # sick_scan_source.hpp includes lidar/source.hpp
    PRIVATE ${SICK_SCAN_XD_LIBRARY}  # the vendor .so stays invisible outside this one .cpp
  )
endif()