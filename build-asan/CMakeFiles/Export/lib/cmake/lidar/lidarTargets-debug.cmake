#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lidar::lidar" for configuration "Debug"
set_property(TARGET lidar::lidar APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(lidar::lidar PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/liblidar.so.0.1.0"
  IMPORTED_SONAME_DEBUG "liblidar.so.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS lidar::lidar )
list(APPEND _IMPORT_CHECK_FILES_FOR_lidar::lidar "${_IMPORT_PREFIX}/lib/liblidar.so.0.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
