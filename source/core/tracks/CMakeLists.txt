add_library(tracks OBJECT
  LineTrack/LineTrack.cpp
  PolygonTrack/PolygonTrack.cpp
  RoundTrack/RoundTrack.cpp
)

set(TRACKS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/LineTrack
  ${CMAKE_CURRENT_SOURCE_DIR}/PolygonTrack
  ${CMAKE_CURRENT_SOURCE_DIR}/RoundTrack
  PARENT_SCOPE
)

target_include_directories(tracks PUBLIC
  "${TRACKS_INCLUDE_DIRS}"
)
