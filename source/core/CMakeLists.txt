find_package(OpenCV REQUIRED)
find_package(Matplot++ REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${GLAD_INCLUDE_DIR}
)

add_subdirectory(tracks)
add_subdirectory(options)
add_subdirectory(fileio)
add_subdirectory(calibration)
add_subdirectory(tracking)

set(CORE_INCLUDE_DIRS
  ${TRACKS_INCLUDE_DIRS}
  ${OPTIONS_INCLUDE_DIRS}
  ${FILEIO_INCLUDE_DIRS}
  ${CALIBRATION_INCLUDE_DIRS}
  ${TRACKING_INCLUDE_DIRS}
  PARENT_SCOPE
)

add_library(core
  $<TARGET_OBJECTS:tracks>
  $<TARGET_OBJECTS:options>
  $<TARGET_OBJECTS:fileio>
  $<TARGET_OBJECTS:calibration>
  $<TARGET_OBJECTS:tracking>
)

target_include_directories(core PUBLIC "${CORE_INCLUDE_DIRS}")

target_link_libraries(core PUBLIC
  ${OpenCV_LIBS}
  opencv_highgui
  opencv_objdetect
  opencv_calib3d
  opencv_aruco
  Matplot++::matplot
  glad
)

target_compile_options(core PUBLIC -g)
