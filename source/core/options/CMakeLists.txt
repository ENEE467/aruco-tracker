add_library(options OBJECT
  BoardMarkers/BoardMarkers.cpp
  CalibrationBoard/CalibrationBoard.cpp
  CameraIntrinsic/CameraIntrinsic.cpp
  LineFollowerMarker/LineFollowerMarker.cpp
  MarkerDetection/MarkerDetection.cpp
)

set(OPTIONS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/BoardMarkers
  ${CMAKE_CURRENT_SOURCE_DIR}/CalibrationBoard
  ${CMAKE_CURRENT_SOURCE_DIR}/CameraIntrinsic
  ${CMAKE_CURRENT_SOURCE_DIR}/LineFollowerMarker
  ${CMAKE_CURRENT_SOURCE_DIR}/MarkerDetection
  PARENT_SCOPE
)

target_include_directories(options PUBLIC
  "${OPTIONS_INCLUDE_DIRS}"
  "${TRACKS_INCLUDE_DIRS}"
)
