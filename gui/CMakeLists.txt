# The GUI is deliberately isolated from the sensor SDK. It subscribes through
# lidar::CloudSubscriber, so it can run on the sensor computer or another PC.
find_package(Qt5 5.15 COMPONENTS Widgets OpenGL QUIET)

if(NOT Qt5_FOUND)
  message(WARNING "Qt5 Widgets/OpenGL not found: cloud_gui will not be built")
  return()
endif()

add_executable(cloud_gui cloud_gui_main.cpp)

target_compile_options(cloud_gui PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wconversion
  -Wshadow
)

target_link_libraries(cloud_gui PRIVATE
  lidar::lidar
  Qt5::Widgets
  Qt5::OpenGL
)

set_target_properties(cloud_gui PROPERTIES
  AUTOMOC ON
)
