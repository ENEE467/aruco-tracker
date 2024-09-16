add_library(fileio OBJECT
  ConfigFile/ConfigFile.cpp
  CSVFile/CSVFile.cpp
  ErrorsPlotFile/ErrorsPlotFile.cpp
  File/File.cpp
  PositionsPlotFile/PositionsPlotFile.cpp
)

set(FILEIO_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ConfigFile
  ${CMAKE_CURRENT_SOURCE_DIR}/CSVFile
  ${CMAKE_CURRENT_SOURCE_DIR}/ErrorsPlotFile
  ${CMAKE_CURRENT_SOURCE_DIR}/File
  ${CMAKE_CURRENT_SOURCE_DIR}/PositionsPlotFile
  PARENT_SCOPE
)

target_include_directories(fileio PUBLIC
  "${FILEIO_INCLUDE_DIRS}"
  "${OPTIONS_INCLUDE_DIRS}"
  "${TRACKS_INCLUDE_DIRS}"
)
