#pragma once

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/core/persistence.hpp>

namespace options {

struct LineFollowerMarker {

  float markerSideMeters {0};
  int markerID {0};
  int markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12};

  LineFollowerMarker() {}
  LineFollowerMarker(const cv::FileStorage& cvFileObjectOut) {readFromConfigFile(cvFileObjectOut);}

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn);
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut);

};

}
