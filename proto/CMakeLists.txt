# ── the generated protobuf code ─────────────────────────────────────────────
#
# protoc turns mw/*.proto into .pb.cc and .pb.h inside the BUILD directory.
# Generated files are build output: they never go in the source tree, because
# two copies of the same truth always drift.

find_package(Protobuf REQUIRED)

# An OBJECT library: the compiled objects get folded into whatever links it,
# so there is no separate libmw_proto.a to install and keep in step.
add_library(mw_proto OBJECT
  mw/envelope.proto
  mw/heartbeat.proto
)

# protoc does not create its own output directory. Forgetting this is the
# classic first failure when wiring protobuf into CMake.
set(MW_PROTO_OUT ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${MW_PROTO_OUT})

# PUBLIC, because mw/message.hpp includes mw/envelope.pb.h, so anyone who
# includes our headers needs to be able to find the generated ones too.
target_include_directories(mw_proto PUBLIC ${MW_PROTO_OUT})

protobuf_generate(
  TARGET         mw_proto
  IMPORT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${MW_PROTO_OUT}
)

# Note what is absent: mw_warnings. Generated code is not ours, it will not
# survive -Werror, and we are not going to fix protoc's output. This is the
# only place in the project that exception is made.
target_link_libraries(mw_proto PUBLIC protobuf::libprotobuf)

# Position-independent code, so the objects can be folded into a shared library
# later without rebuilding them.
set_target_properties(mw_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)


add_library(lidar_proto OBJECT lidar_scan.proto lidar_point_cloud.proto)

set(LIDAR_PROTO_OUT ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${LIDAR_PROTO_OUT})
target_include_directories(lidar_proto PUBLIC ${LIDAR_PROTO_OUT})

protobuf_generate(
  TARGET         lidar_proto
  IMPORT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${LIDAR_PROTO_OUT}
)

target_link_libraries(lidar_proto PUBLIC protobuf::libprotobuf)
set_target_properties(lidar_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(imu_proto OBJECT imu.proto)

set(IMU_PROTO_OUT ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${IMU_PROTO_OUT})
target_include_directories(imu_proto PUBLIC ${IMU_PROTO_OUT})

protobuf_generate(
  TARGET         imu_proto
  IMPORT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${IMU_PROTO_OUT}
)

target_link_libraries(imu_proto PUBLIC protobuf::libprotobuf)
set_target_properties(imu_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)


add_library(camera_proto OBJECT camera_frame.proto)

set(CAMERA_PROTO_OUT ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CAMERA_PROTO_OUT})
target_include_directories(camera_proto PUBLIC ${CAMERA_PROTO_OUT})

protobuf_generate(
  TARGET         camera_proto
  IMPORT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${CAMERA_PROTO_OUT}
)

target_link_libraries(camera_proto PUBLIC protobuf::libprotobuf)
set_target_properties(camera_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(nav_proto OBJECT nav_command.proto nav_waypoint.proto)

set(NAV_PROTO_OUT ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${NAV_PROTO_OUT})
target_include_directories(nav_proto PUBLIC ${NAV_PROTO_OUT})

protobuf_generate(
  TARGET         nav_proto
  IMPORT_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${NAV_PROTO_OUT}
)

target_link_libraries(nav_proto PUBLIC protobuf::libprotobuf)
set_target_properties(nav_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)
