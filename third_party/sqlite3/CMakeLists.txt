# Vendored SQLite -- the "amalgamation," a single .c/.h pair straight from
# sqlite.org with everything already inlined. Compiling it ourselves means
# nothing on this machine needs sqlite3 installed system-wide; the exact
# same source ships with the project and builds the same way everywhere.
add_library(sqlite3_vendored STATIC
  sqlite3.c
)

target_include_directories(sqlite3_vendored PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Keeps this vendored copy from ever needing libdl -- SQLite only uses it
# for loading runtime extensions (.so plugins), which we never do.
target_compile_definitions(sqlite3_vendored PRIVATE
  SQLITE_OMIT_LOAD_EXTENSION
)

find_package(Threads REQUIRED)  # always available with any toolchain --
target_link_libraries(sqlite3_vendored PUBLIC Threads::Threads)  # not a package install

# Same name (SQLite::SQLite3) that find_package(SQLite3) would have
# produced -- so src/CMakeLists.txt and tests/CMakeLists.txt need zero
# changes; they already link against this exact target name.
add_library(SQLite::SQLite3 ALIAS sqlite3_vendored)

# Gets folded into liblidar.so, so it must be PIC too.
set_target_properties(sqlite3_vendored PROPERTIES POSITION_INDEPENDENT_CODE ON)