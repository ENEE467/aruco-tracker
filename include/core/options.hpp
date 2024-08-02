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
    markerIDs {0, 1, 2, 3},
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

class RoundTrack {

public:
  RoundTrack()
  : _center {0, 0},
    _majorAxisLength {0},
    _minorAxisLength {0},
    _a {0},
    _b {0},
    _aInv {0},
    _bInv {0},
    _evoluteXCalcMidPart {0},
    _evoluteYCalcMidPart {0}
  {}

  RoundTrack(double xIn, double yIn, double majorAxisLengthIn, double minorAxisLengthIn)
  : _center {xIn, yIn},
    _majorAxisLength {majorAxisLengthIn},
    _minorAxisLength {minorAxisLengthIn}
  {
    updateParameters();
  }

  void setParameters(const cv::Point2d& centerIn, double majorAxisLengthIn, double minorAxisLengthIn)
  {
    _center = centerIn;
    _majorAxisLength = majorAxisLengthIn;
    _minorAxisLength = minorAxisLengthIn;

    updateParameters();
  }

  const cv::Point2d& getCenter() const {return _center;}
  const double getMajorAxisLength() const {return _majorAxisLength;}
  const double getMinorAxisLength() const {return _minorAxisLength;}
  const double getSemiMajorAxisLength() const {return _a;}
  const double getSemiMinorAxisLength() const {return _b;}

  const double calculatePerpendicularDistance(const cv::Point2d& positionIn) const
  {
    double perpendicularDistance {0.0};

    if (_a <= 0 || _b <= 0)
      return perpendicularDistance;

    double evoluteT {
      std::atan2((positionIn.y - _center.y) * _bInv, (positionIn.x - _center.x) * _aInv)};

    double sinEvoluteT {std::sin(evoluteT)};
    double cosEvoluteT {std::cos(evoluteT)};

    double evoluteX {_center.x + _evoluteXCalcMidPart * std::pow(cosEvoluteT, 3)};
    double evoluteY {_center.y + _evoluteYCalcMidPart * std::pow(sinEvoluteT, 3)};

    double radiusOfCurvature {
      std::pow(_b*_b * cosEvoluteT*cosEvoluteT + _a*_a * sinEvoluteT*sinEvoluteT, 1.5) * (_aInv*_bInv)};

    double distanceToEvolute {std::hypot(positionIn.x - evoluteX, positionIn.y - evoluteY)};

    perpendicularDistance = std::abs(distanceToEvolute - radiusOfCurvature);

    return perpendicularDistance;
  }

private:
  void updateParameters()
  {
    _a = _majorAxisLength / 2;
    _b = _minorAxisLength / 2;
    _aInv = _a ? 1 / _a : 0;
    _bInv = _b ? 1 / _b : 0;
    _evoluteXCalcMidPart = (_a*_a - _b*_b) * _aInv;
    _evoluteYCalcMidPart = (_b*_b - _a*_a) * _bInv;
  }

  cv::Point2d _center;
  double _majorAxisLength;
  double _minorAxisLength;

  double _a;
  double _b;
  double _aInv;
  double _bInv;
  double _evoluteXCalcMidPart;
  double _evoluteYCalcMidPart;

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
