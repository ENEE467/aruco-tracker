add_library(calibration OBJECT
  Calibrator/Calibrator.cpp
)

set(CALIBRATION_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/Calibrator
  PARENT_SCOPE
)

target_include_directories(calibration PUBLIC
  "${CALIBRATION_INCLUDE_DIRS}"
  "${OPTIONS_INCLUDE_DIRS}"
)
