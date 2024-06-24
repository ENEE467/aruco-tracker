#pragma once

#include <vector>
#include <opencv2/objdetect/aruco_detector.hpp>

namespace options {

struct MarkerDetection {

  int camID;
  int frameWidthPixels;
  int frameHeightPixels;
  int frameRateFPS;
  bool showRejectedMarkers;
  cv::aruco::DetectorParameters detectorParameters;

  MarkerDetection()
  : camID {0},
    frameWidthPixels {640},
    frameHeightPixels {480},
    frameRateFPS {30},
    showRejectedMarkers {false},
    detectorParameters {cv::aruco::DetectorParameters()}
  {}

};

// TODO: Integrate this struct into the program
struct LineFollowerMarker {
  LineFollowerMarker()
  : markerSideMeters {0},
    markerID {0},
    markerDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL}
  {}

  float markerSideMeters;
  int markerID;
  int markerDictionaryID;
};

struct BoardMarkers {
  BoardMarkers()
  : markerSideMeters {0},
    markerSeperationMetersX {0},
    markerSeperationMetersY {0},
    markerIDs {},
    markerDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL}
  {}

  float markerSideMeters;
  float markerSeperationMetersX;
  float markerSeperationMetersY;
  std::vector<int> markerIDs;
  int markerDictionaryID;
};

struct Calibration {
  Calibration()
  : camID {0},
    inputFilePath {"none"},
    markerSideMeters {0},
    squareSideMeters {0},
    squaresQuantityX {0},
    squaresQuantityY {0},
    markerDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL}
    {}

  int camID;
  cv::String inputFilePath;
  float markerSideMeters;
  float squareSideMeters;
  int squaresQuantityX;
  int squaresQuantityY;
  int markerDictionaryID;
};

struct CalibrationOutput {
  CalibrationOutput()
  : cameraMatrix {cv::Mat::zeros(cv::Size(3, 3), CV_32F)},
    distCoeffs {cv::Mat::zeros(cv::Size(5, 1), CV_32F)}
    {}

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
};

}
