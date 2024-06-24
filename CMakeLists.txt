cmake_minimum_required(VERSION 3.11)
project(LineFollowerTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-lopencv_highgui)
add_compile_options(-lopencv_objdetect)
add_compile_options(-lopencv_calib3d)
add_compile_options(-lopencv_aruco)

add_executable(line-follower-tracker
  src/fileio.cpp
  src/tracker.cpp
  src/main.cpp)

target_include_directories(line-follower-tracker PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(line-follower-tracker ${OpenCV_LIBS} Eigen3::Eigen)
