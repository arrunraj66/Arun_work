# -----------------------------------------------------------------------------
#  Custom AUV middleware -- top level build description
#
#  Stage 1 of the build plan: a reproducible build with every warning on and
#  four sanitiser configurations available, before there is anything to build.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22)

project(auv_middleware
        VERSION      0.1.0
        DESCRIPTION  "Custom middleware for AUV onboard software"
        LANGUAGES    CXX C)

# -- language level ----------------------------------------------------------
set(CMAKE_CXX_STANDARD          20)   # we want C++20, not "at least C++20"
set(CMAKE_CXX_STANDARD_REQUIRED ON)   # fail loudly rather than falling back
set(CMAKE_CXX_EXTENSIONS        OFF)  # -std=c++20, never -std=gnu++20

# -- a default build type, so a bare `cmake -S . -B build` is not undefined --
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Debug | Release | RelWithDebInfo" FORCE)
endif()

# -- compile_commands.json, for clangd / clang-tidy / your editor ------------
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -- where our own .cmake modules live ---------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Warnings)
include(Sanitizers)
include(ZeroMQ)
include(OpenCV)
include(SickScanXd)
add_subdirectory(third_party/sqlite3)  # vendored SQLite amalgamation -- see
                                        # third_party/sqlite3/CMakeLists.txt.
                                        # Produces the same SQLite::SQLite3
                                        # target find_package(SQLite3) would
                                        # have, so nothing downstream changes.
# -- the pieces --------------------------------------------------------------
add_subdirectory(proto)   # must come first: src/ links the generated code
add_subdirectory(src)
add_subdirectory(apps)

# Qt point-cloud viewer. The gui/CMakeLists.txt quietly skips this target when
# Qt5 development packages are not installed, so headless builds still work.
option(MW_BUILD_GUI "Build the Qt 3D point-cloud viewer" ON)
if(MW_BUILD_GUI)
  add_subdirectory(gui)
endif()
# Step 8: install rules + the lidarConfig.cmake that find_package() looks for.
# Must come after add_subdirectory(src) -- it refers to the `lidar` target.
include(InstallLidar)
include(CTest)                        # defines BUILD_TESTING, default ON
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# -- a one-line summary at configure time, so nobody builds the wrong thing --
message(STATUS "--------------------------------------------------")
message(STATUS "  ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  sanitiser  : ${MW_SANITIZER}")
message(STATUS "  warnings   : ${MW_WARNINGS_AS_ERRORS}")
message(STATUS "--------------------------------------------------")