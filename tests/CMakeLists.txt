# One executable per test file. Each is a program that exits non-zero on
# failure; ctest needs nothing more than that.
foreach(test_name IN ITEMS test_version test_message test_transport test_acoustic test_node)
  add_executable(${test_name} ${test_name}.cpp)
  target_link_libraries(${test_name} PRIVATE mw::mw mw_warnings)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
# Runs a server on its own std::thread, so it needs Threads::Threads
# explicitly -- the only test in this file that does.
find_package(Threads REQUIRED)
add_executable(test_reqrep test_reqrep.cpp)
target_link_libraries(test_reqrep PRIVATE mw::mw mw_warnings Threads::Threads)
add_test(NAME test_reqrep COMMAND test_reqrep)
# The backup query path, proven end to end: ScanRecorder seeds a real
# recording, ScanQueryServer answers from it, ScanQueryClient asks. Public
# lidar:: API only. Runs the server on its own thread, hence Threads::Threads
# (find_package(Threads) already ran for test_reqrep above).
add_executable(test_query test_query.cpp)
target_compile_options(test_query PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_query PRIVATE lidar::lidar Threads::Threads)
add_test(NAME test_query COMMAND test_query)

add_executable(test_scan_codec test_scan_codec.cpp)
target_include_directories(test_scan_codec PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(test_scan_codec PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
# This test reaches into src/ on purpose -- it calls to_proto()/from_proto()
# directly, which the public lidar:: target never exposes. It therefore also
# needs lidar_proto's generated headers directly; a normal consumer of
# lidar::lidar never would, and never gets them, because that dependency is
# PRIVATE on the lidar target itself.
target_link_libraries(test_scan_codec PRIVATE lidar::lidar lidar_proto)
add_test(NAME test_scan_codec COMMAND test_scan_codec)

# Reaches into src/ (scan_codec.hpp) and links sqlite3 directly, the same
# reasoning as test_scan_codec above -- this test reads the log/index the
# way a real reader will, not through any public lidar:: API.
add_executable(test_recorder test_recorder.cpp)
target_include_directories(test_recorder PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(test_recorder PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_recorder PRIVATE lidar::lidar lidar_proto SQLite::SQLite3)
add_test(NAME test_recorder COMMAND test_recorder)

# Step 7's own test, deliberately using ONLY the public lidar:: API (no
# src/ include directory, no lidar_proto, no SQLite::SQLite3 here) -- that's
# the point of ScanReader existing: a caller records and reads scans back
# without ever knowing protobuf or SQLite are involved.
add_executable(test_reader test_reader.cpp)
target_compile_options(test_reader PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_reader PRIVATE lidar::lidar)
add_test(NAME test_reader COMMAND test_reader)

# Steps 4+5 can only be tested together, so they are. Uses ONLY the public
# lidar:: API -- no src/ include dir, no lidar_proto, no ZeroMQ, no mw
# headers -- which is itself part of what the test proves.
add_executable(test_pubsub test_pubsub.cpp)
target_compile_options(test_pubsub PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_pubsub PRIVATE lidar::lidar)
add_test(NAME test_pubsub COMMAND test_pubsub)



# 3D counterpart of test_scan_codec: reaches into src/ for cloud_codec.hpp
# and needs lidar_proto's generated headers, same reasoning as above.
add_executable(test_cloud_codec test_cloud_codec.cpp)
target_include_directories(test_cloud_codec PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_compile_options(test_cloud_codec PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
target_link_libraries(test_cloud_codec PRIVATE lidar::lidar lidar_proto)
add_test(NAME test_cloud_codec COMMAND test_cloud_codec)