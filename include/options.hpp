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

  float markerSideMeters;
  int markerID;
  int markerDictionaryID;

  LineFollowerMarker()
  : markerSideMeters {0},
    markerID {0},
    markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12}
  {}

};

struct BoardMarkers {

  float markerSideMeters;
  float markerSeperationMetersX;
  float markerSeperationMetersY;
  std::vector<int> markerIDs;
  int markerDictionaryID;

  BoardMarkers()
  : markerSideMeters {0},
    markerSeperationMetersX {0},
    markerSeperationMetersY {0},
    markerIDs {},
    markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12}
  {}

};

struct LineTrack {

  cv::Point2d point1;
  cv::Point2d point2;
  double length;
  double length_inv;

  LineTrack()
  : point1 {0, 0},
    point2 {0, 0},
    length {0},
    length_inv {0}
  {}

  LineTrack(double x1In, double y1In, double x2In, double y2In)
  : point1 {x1In, y1In},
    point2 {x2In, y2In},
    length {std::hypot(x2In - x1In, y2In - y1In)},
    length_inv {length ? 1 / length : 0}
  {}

  void updateLength()
  {
    length = std::hypot(point2.x - point1.x, point2.y - point1.y);
    length_inv = length ? 1 / length : 0;
  }

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const
  {
    double y_diff {point2.y - point1.y};
    double x_diff {point2.x - point1.x};
    double c {point2.x * point1.y - point2.y * point1.x};

    return std::abs(y_diff * positionIn.x - x_diff * positionIn.y + c) * length_inv;
  }

};

struct RoundTrack {

  cv::Point2d center;
  double majorAxis;
  double minorAxis;

  RoundTrack()
  : center {0, 0},
    majorAxis {0},
    minorAxis {0}
  {}

  RoundTrack(double xIn, double yIn, double majorAxisIn, double minorAxisIn)
  : center {xIn, yIn},
    majorAxis {majorAxisIn},
    minorAxis {minorAxisIn}
  {}

  double semiMajorAxis() const {return majorAxis * 0.5;}
  double semiMinorAxis() const {return minorAxis * 0.5;}

};

enum class TrackSelection {

  ROUND = 0,
  LINE = 1

};

struct Track {

  TrackSelection selection;
  LineTrack lineTrack;
  RoundTrack roundTrack;

  Track()
  : selection {TrackSelection::LINE},
    lineTrack {},
    roundTrack {}
  {}

};

struct CalibrationBoard {

  float markerSideMeters;
  int markerDictionaryID;
  float squareSideMeters;
  int squaresQuantityX;
  int squaresQuantityY;

  CalibrationBoard()
  : markerSideMeters {0},
    markerDictionaryID {cv::aruco::DICT_ARUCO_MIP_36h12},
    squareSideMeters {0},
    squaresQuantityX {0},
    squaresQuantityY {0}
  {}

};

struct CameraIntrinsic {

public:
  cv::Mat cameraMatrix;
  cv::Mat distortionCoefficients;

  CameraIntrinsic()
  : cameraMatrix {cv::Mat::zeros(cv::Size(3, 3), CV_32F)},
    distortionCoefficients {cv::Mat::zeros(cv::Size(5, 1), CV_32F)},
    _isNonZero {isNonZeroMatrix(cameraMatrix) && isNonZeroMatrix(distortionCoefficients)}
  {}

  const bool isNonZero() const {return _isNonZero;}

  void evaluateNonZero()
  {
    _isNonZero = (isNonZeroMatrix(cameraMatrix) && isNonZeroMatrix(distortionCoefficients));
  }

private:
  bool _isNonZero;

  bool isNonZeroMatrix(const cv::Mat& matrix)
  {
    cv::Mat zeroMatrix {cv::Mat::zeros(matrix.rows, matrix.cols, matrix.type())};
    auto comparisonMatrix {matrix != zeroMatrix};

    return cv::countNonZero(comparisonMatrix);
  }

};
};

}
