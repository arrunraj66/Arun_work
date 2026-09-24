# OpenCV, for the camera side (camera_publisher_demo and whatever's built on
# top of it later). Only the three components we actually use: core (cv::Mat),
# imgcodecs (cv::imencode -- JPEG), videoio (cv::VideoCapture -- the webcam).
# Not the whole of OpenCV (no calib3d, no DNN, none of that) -- same instinct
# as ZeroMQ.cmake: bring in exactly what the code links against.
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)

# An INTERFACE target so the rest of the tree says `mw_opencv` and never has
# to know it was found via find_package(OpenCV) specifically.
add_library(mw_opencv INTERFACE)
target_link_libraries(mw_opencv INTERFACE ${OpenCV_LIBS})
# SYSTEM: OpenCV's own headers are not our code, and we do not want
# -Wconversion / -Wpedantic turning someone else's header into a build error
# for us (same reasoning as mw_zeromq in ZeroMQ.cmake).
target_include_directories(mw_opencv SYSTEM INTERFACE ${OpenCV_INCLUDE_DIRS})

message(STATUS "  opencv     : ${OpenCV_VERSION}")
