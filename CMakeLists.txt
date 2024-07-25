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

###############################################################################
# UNIT TESTS
###############################################################################

add_subdirectory(test)

###############################################################################
# IMGUI
###############################################################################

# GLAD
add_subdirectory(include/glad)

# Path to ImGui source
set (IMGUI_SOURCE_PATH libs/imgui)

set(IMGUI_HEADERS
    ${IMGUI_SOURCE_PATH}/imconfig.h
    ${IMGUI_SOURCE_PATH}/imgui_internal.h
    ${IMGUI_SOURCE_PATH}/imgui.h
    ${IMGUI_SOURCE_PATH}/imstb_rectpack.h
    ${IMGUI_SOURCE_PATH}/imstb_textedit.h
    ${IMGUI_SOURCE_PATH}/imstb_truetype.h
    ${IMGUI_SOURCE_PATH}/backends/imgui_impl_glfw.h
    ${IMGUI_SOURCE_PATH}/backends/imgui_impl_opengl3.h
    ${IMGUI_SOURCE_PATH}/backends/imgui_impl_opengl3_loader.h
    ${IMGUI_SOURCE_PATH}/misc/cpp/imgui_stdlib.h
)

set(IMGUI_SRC
    ${IMGUI_SOURCE_PATH}/imgui_demo.cpp
    ${IMGUI_SOURCE_PATH}/imgui_draw.cpp
    ${IMGUI_SOURCE_PATH}/imgui_tables.cpp
    ${IMGUI_SOURCE_PATH}/imgui_widgets.cpp
    ${IMGUI_SOURCE_PATH}/imgui.cpp
    ${IMGUI_SOURCE_PATH}/backends/imgui_impl_glfw.cpp
    ${IMGUI_SOURCE_PATH}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_SOURCE_PATH}/misc/cpp/imgui_stdlib.cpp
)

find_package(OpenGL REQUIRED)

add_library(imgui ${IMGUI_HEADERS} ${IMGUI_SRC})
if(MSVC)
  target_compile_options(imgui PRIVATE /W4 /WX /arch:AVX2 /fp:fast)
endif()

if(WIN32)
  target_link_libraries(imgui PUBLIC glfw glad OpenGL::GL imm32)
else()
  target_link_libraries(imgui PUBLIC glfw glad OpenGL::GL)
endif()

target_compile_definitions(imgui PRIVATE IMGUI_DLL_EXPORT)

include_directories(
  ${IMGUI_SOURCE_PATH}/
  ${IMGUI_SOURCE_PATH}/examples/libs/gl3w
  ${IMGUI_SOURCE_PATH}/backends
  ${IMGUI_SOURCE_PATH}/misc/cpp
)
