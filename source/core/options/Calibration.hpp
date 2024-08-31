#pragma once

#include "MarkerDetection/MarkerDetection.hpp"
#include "CalibrationBoard/CalibrationBoard.hpp"
#include "CameraIntrinsic/CameraIntrinsic.hpp"

namespace options {

struct Calibration {

  MarkerDetection detection;
  CalibrationBoard calibrationBoard;
  CameraIntrinsic intrinsicParams;

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
  {
    detection.readFromConfigFile(cvFileObjectIn);
    calibrationBoard.readFromConfigFile(cvFileObjectIn);
    intrinsicParams.readFromConfigFile(cvFileObjectIn);
  }

};

}
