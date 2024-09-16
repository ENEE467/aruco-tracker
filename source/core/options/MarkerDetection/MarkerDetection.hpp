#pragma once

#include <opencv2/objdetect/aruco_detector.hpp>

namespace options {

struct MarkerDetection {

  int camID {0};
  int frameWidthPixels {640};
  int frameHeightPixels {480};
  int frameRateFPS {30};
  bool showRejectedMarkers {false};
  cv::aruco::DetectorParameters detectorParameters{cv::aruco::DetectorParameters()};

  MarkerDetection() {}
  MarkerDetection(const cv::FileStorage& cvFileObjectIn) {readFromConfigFile(cvFileObjectIn);}

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn);
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut);
};

}
