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

class LineTrack {

public:
  LineTrack()
  : _point1 {0, 0},
    _point2 {0, 0},
    _length {0},
    _lengthInv {0}
  {}

  LineTrack(double x1In, double y1In, double x2In, double y2In)
  : _point1 {x1In, y1In},
    _point2 {x2In, y2In}
  {
    updateLength();
  }

  void setPoints(const cv::Point2d& point1In, const cv::Point2d& point2In)
  {
    _point1 = point1In;
    _point2 = point2In;

    updateLength();
  }

  const cv::Point2d& getPoint1() const {return _point1;}
  const cv::Point2d& getPoint2() const {return _point2;}
  const double getLength() const {return _length;}

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const
  {
    double perpendicularDistance {0.0};

    if (_length <= 0)
      return perpendicularDistance;

    double yDiff {_point2.y - _point1.y};
    double xDiff {_point2.x - _point1.x};
    double c {_point2.x * _point1.y - _point2.y * _point1.x};

    perpendicularDistance = std::abs(yDiff * positionIn.x - xDiff * positionIn.y + c) * _lengthInv;

    return perpendicularDistance;
  }

private:
  void updateLength()
  {
    _length = std::hypot(_point2.x - _point1.x, _point2.y - _point1.y);
    _lengthInv = _length ? 1 / _length : 0;
  }

  cv::Point2d _point1;
  cv::Point2d _point2;
  double _length;
  double _lengthInv;

};
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

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const
  {
    // TODO: Implement this method.
    return 0;
  }

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

struct Tracking {

  MarkerDetection detection;
  LineFollowerMarker lineFollowerMarker;
  BoardMarkers boardMarkers;
  Track track;
  CameraIntrinsic calibrationParams;

};

struct Calibration {

  MarkerDetection detection;
  CalibrationBoard calibrationBoard;
  CameraIntrinsic calibrationParams;

};

}
