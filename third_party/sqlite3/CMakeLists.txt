find_package(Threads REQUIRED)
add_library(sqlite3_vendored STATIC shim.c)
target_link_libraries(sqlite3_vendored PUBLIC Threads::Threads sqlite3)
target_include_directories(sqlite3_vendored PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
add_library(SQLite::SQLite3 ALIAS sqlite3_vendored)

# Gets folded into liblidar.so, so it must be PIC too.
set_target_properties(sqlite3_vendored PROPERTIES POSITION_INDEPENDENT_CODE ON)
