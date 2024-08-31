#pragma once

#include <vector>

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/core/persistence.hpp>

namespace options {

struct BoardMarkers {

  float markerSideMeters {0};
  float markerSeperationMetersX {0};
  float markerSeperationMetersY {0};
  std::vector<int> markerIDs {};
  int markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12};

  BoardMarkers() {}
  BoardMarkers(const cv::FileStorage& cvFileObjectIn) {readFromConfigFile(cvFileObjectIn);}

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn);
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut);

};

}
