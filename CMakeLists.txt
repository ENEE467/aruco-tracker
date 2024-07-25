cmake_minimum_required(VERSION 3.11)
project(LineFollowerTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# GoogleTest integration
include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
###############################################################################
# CORE
###############################################################################

find_package(OpenCV REQUIRED)
find_package(Matplot++ REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-lopencv_highgui)
add_compile_options(-lopencv_objdetect)
add_compile_options(-lopencv_calib3d)
add_compile_options(-lopencv_aruco)

add_library(core
  src/core/calibration.cpp
  src/core/fileio.cpp
  src/core/plotting.cpp
  src/core/tracking.cpp
  # src/core/main.cpp
)

target_include_directories(core PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(core
  ${OpenCV_LIBS}
  Matplot++::matplot
)

add_subdirectory(test)
