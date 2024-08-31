#pragma once

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/core/persistence.hpp>

namespace options {

struct CalibrationBoard {

  float markerSideMeters {0};
  int markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12};
  float squareSideMeters {0};
  int squaresQuantityX {0};
  int squaresQuantityY {0};

  CalibrationBoard() {}
  CalibrationBoard(const cv::FileStorage& cvFileObjectIn) {readFromConfigFile(cvFileObjectIn);}

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn);
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut);

};

}
