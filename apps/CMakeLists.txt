foreach(app_name IN ITEMS mw_info mw_acoustic_demo mw_msgdemo mw_pub mw_sub mw_executor_demo scan_step1_demo)
  add_executable(${app_name} ${app_name}.cpp)
  target_link_libraries(${app_name} PRIVATE mw::mw mw_warnings)
endforeach()
# Step 5's runnable half. No sensor SDK needed -- it only subscribes, so it
# builds and runs on any machine, including one with no picoScan150 attached.
add_executable(scan_subscriber_main scan_subscriber_main.cpp)
target_compile_options(scan_subscriber_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(scan_subscriber_main PRIVATE lidar::lidar)

# Step 6's (3D) subscribing half, same reasoning as scan_subscriber_main
# above: no sensor SDK needed, builds and runs anywhere.
add_executable(cloud_subscriber_main cloud_subscriber_main.cpp)
target_compile_options(cloud_subscriber_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(cloud_subscriber_main PRIVATE lidar::lidar)
# The backup query path. Neither half needs the sensor SDK -- the server
# only reads a log+index that already exists, so both build and run on any
# machine, same reasoning as scan_subscriber_main above.
add_executable(scan_query_server_main scan_query_server_main.cpp)
target_compile_options(scan_query_server_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(scan_query_server_main PRIVATE lidar::lidar)

#add_executable(scan_query_client_demo scan_query_client_demo.cpp)
#target_compile_options(scan_query_client_demo PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
#target_link_libraries(scan_query_client_demo PRIVATE lidar::lidar)
# The camera side's first step, same shape as sick_scan_source_demo was for
# the lidar: a standalone program proving the webcam + publish path work in
# isolation, before any of it becomes a library or a supervised service.
# Links mw::mw directly (not lidar::lidar) -- it only needs the transport
# layer, nothing lidar-specific.
add_executable(camera_publisher_demo camera_publisher_demo.cpp)
target_compile_options(camera_publisher_demo PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(camera_publisher_demo PRIVATE mw::mw mw_opencv)
# Ram's own single-command dashboard: watches the live stream, the SQLite
# index, and the localhost query backup all at once. No sensor SDK needed --
# same reasoning as scan_query_server_main above.
add_executable(lidar_monitor_main lidar_monitor_main.cpp)
target_compile_options(lidar_monitor_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(lidar_monitor_main PRIVATE lidar::lidar)
# The actual runnable "node" for SickScanSource
if(SICK_SCAN_XD_FOUND)
  add_executable(sick_scan_source_demo sick_scan_source_demo.cpp)
  target_include_directories(sick_scan_source_demo PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(sick_scan_source_demo PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
    target_link_libraries(sick_scan_source_demo PRIVATE lidar::sick)

  # Step 9 (live sensor) + Step 6 (recorder) wired together. Needs both
  # lidar::sick (for SickScanSource) and lidar::lidar (for ScanRecorder);
  # lidar::sick already pulls lidar::lidar in PUBLIC-ly, but naming both
  # says what this program actually uses rather than relying on that.
  add_executable(record_live_scans record_live_scans.cpp)
  target_include_directories(record_live_scans PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(record_live_scans PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(record_live_scans PRIVATE lidar::sick lidar::lidar)
    # The real handoff program: publish live AND record to log+index from one
  # sensor connection. Needs lidar::sick (for SickScanSource, same reason as
  # record_live_scans) and lidar::lidar (for ScanRecorder + ScanPublisher).
  add_executable(scan_publisher_recorder_main scan_publisher_recorder_main.cpp)
  target_include_directories(scan_publisher_recorder_main PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(scan_publisher_recorder_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(scan_publisher_recorder_main PRIVATE lidar::sick lidar::lidar)

  # Step 4's runnable half: sensor in, network out. Needs the SDK, so it
  # lives inside this guard alongside the other sensor-owning programs.
  add_executable(scan_publisher_main scan_publisher_main.cpp)
  target_include_directories(scan_publisher_main PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(scan_publisher_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(scan_publisher_main PRIVATE lidar::sick lidar::lidar)
    # Step 6's (3D) publishing half: sensor in, network out, same reasoning as
  # scan_publisher_main above. Needs lidar::sick for SickCloudSource and
  # lidar::lidar for CloudPublisher.
  add_executable(cloud_publisher_main cloud_publisher_main.cpp)
  target_include_directories(cloud_publisher_main PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(cloud_publisher_main PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(cloud_publisher_main PRIVATE lidar::sick lidar::lidar)
endif()
add_executable(record_sample_data record_sample_data.cpp)
target_compile_options(record_sample_data PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(record_sample_data PRIVATE lidar::lidar)s
# Reads back whatever record_sample_data (or a real recording) produced.
# Only touches lidar::lidar's PUBLIC API (ScanReader) -- same reasoning as
# test_reader: proves the log+index pair is usable without protobuf or
# SQLite ever appearing in a caller's own code.
add_executable(dump_scans dump_scans.cpp)
target_compile_options(dump_scans PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(dump_scans PRIVATE lidar::lidar)

  add_executable(sick_cloud_source_demo sick_cloud_source_demo.cpp)
  target_include_directories(sick_cloud_source_demo PRIVATE ${PROJECT_SOURCE_DIR}/src)
  target_compile_options(sick_cloud_source_demo PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  target_link_libraries(sick_cloud_source_demo PRIVATE lidar::sick)