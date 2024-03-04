cmake_minimum_required(VERSION 2.8)
project( DetectMarkers )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-lopencv_highgui)
add_compile_options(-lopencv_objdetect)
add_compile_options(-lopencv_calib3d)
add_compile_options(-lopencv_aruco)

add_executable(line-follower-tracker
  src/tracker.cpp
  src/main.cpp)

target_include_directories(line-follower-tracker PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(line-follower-tracker ${OpenCV_LIBS})
