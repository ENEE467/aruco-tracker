add_library(tracking OBJECT
  BoardDetector/BoardDetector.cpp
  LineFollowerDetector/LineFollowerDetector.cpp
  Tracker/Tracker.cpp
  Output.hpp
)

set(TRACKING_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/BoardDetector
  ${CMAKE_CURRENT_SOURCE_DIR}/LineFollowerDetector
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracker
  PARENT_SCOPE
)

target_include_directories(tracking PUBLIC
  "${TRACKING_INCLUDE_DIRS}"
  "${FILEIO_INCLUDE_DIRS}"
  "${OPTIONS_INCLUDE_DIRS}"
  "${TRACKS_INCLUDE_DIRS}"
)
