#include "ArcTrack.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace tracks {

ArcTrack::ArcTrack(const cv::Point2d& point1In, const cv::Point2d& point2In, double sweepDegrees)
: Track(),
  _point1 {point1In},
  _point2 {point2In},
  _sweepAngleRadians {sweepDegrees * (M_PI / 180)}
{
  updateParameters();

  _objectPoints3D.reserve(8);
  _imagePoints2D.reserve(8);

  for (double theta {_angleMinRadians}; theta <= _angleMaxRadians; theta += (_sweepAngleRadians) / 8) {
    _objectPoints3D.emplace_back(
      _center.x + _radius * std::cos(theta),
      _center.y + _radius * std::sin(theta),
      0);
  }

  // _objectPoints3D.emplace_back(_center.x, _center.y, 0);
  // _objectPoints3D.emplace_back(
  //   _center.x + _radius * std::cos(M_PI_2 / 2),
  //   _center.y + _radius * std::sin(M_PI_2 / 2),
  //   0);
}

ArcTrack::ArcTrack(const cv::FileStorage& cvFileObjectIn)
: Track()
{
  readFromConfigFile(cvFileObjectIn);

  _objectPoints3D.reserve(8);
  _imagePoints2D.reserve(8);

  for (double theta {_angleMinRadians}; theta <= _angleMaxRadians; theta += (_sweepAngleRadians) / 8) {
    _objectPoints3D.emplace_back(
      _center.x + _radius * std::cos(theta),
      _center.y + _radius * std::sin(theta),
      0);
  }

  // _objectPoints3D.emplace_back(_center.x, _center.y, 0);
  // _objectPoints3D.push_back(_objectPoints3D.back());
  // _objectPoints3D.emplace_back(_point1.x, _point1.y, 0);
  // _objectPoints3D.emplace_back(_point2.x, _point2.y, 0);

  // _objectPoints3D.reserve(2);
  // _imagePoints2D.reserve(2);

  // _objectPoints3D.emplace_back(_center.x, _center.y, 0);
  // _objectPoints3D.emplace_back(
  //   _center.x + _radius * std::cos(0),
  //   _center.y + _radius * std::sin(0),
  //   0);
}

void ArcTrack::updateParameters()
{
  // Assumes anti-clockwise rotation. Point1 rotates to Point 2 anti-clockwise.
  double distance {std::hypot(_point1.x - _point2.x, _point1.y - _point2.y)};
  double height {(distance * 0.5) / std::tan(_sweepAngleRadians * 0.5)};
  _radius = (distance * 0.5) / std::sin(_sweepAngleRadians * 0.5);

  double xMidpoint {(_point1.x + _point2.x) * 0.5};
  double yMidpoint {(_point1.y + _point2.y) * 0.5};

  // double slopeTwoPoints {(_point2.y - _point1.y) / (_point2.x - _point1.x)};
  // double slopeAngleBisectorRadians {std::atan(-1 / slopeTwoPoints)};
  double slopeAngleBisectorRadians {std::atan2(-(_point2.x - _point1.x), _point2.y - _point1.y)};

  // std::cout << "Angle bisector slope: " << slopeAngleBisectorRadians << '\n';
  // if (_point2.x - _point1.x == 0)
  //   slopeAngleBisectorRadians = 0;
  // else if (_point2.y - _point1.y == 0)
  //   slopeAngleBisectorRadians = M_PI_2;
  // else
  // double slopeAngleBisectorRadians {};
  // slopeAngleBisectorRadians = M_PI_2 + std::atan2(_point2.y - _point1.y, _point2.x - _point1.x);

  _center.x = xMidpoint - height * std::cos(slopeAngleBisectorRadians);
  _center.y = yMidpoint - height * std::sin(slopeAngleBisectorRadians);


  _angleMinRadians = std::atan2(_point1.y - _center.y, _point1.x - _center.x);
  // _angleMinDegrees = _angleMinRadians * (180 / M_PI);

  _angleMaxRadians = _angleMinRadians + _sweepAngleRadians;
  // _angleMaxRadians = std::floor(std::atan2(_point2.y - _center.y, _point2.x - _center.x));
  // _angleMaxDegrees = _angleMaxRadians * (180 / M_PI);
}

double ArcTrack::calculatePerpendicularDistance(const cv::Point2d& positionIn) const
{
  double distance {0};
  double slopeAnglePosition2Center {
    std::atan2(positionIn.y - _center.y, positionIn.x - _center.x)};

  if (slopeAnglePosition2Center < _angleMinRadians)
    distance = std::hypot(positionIn.x - _point1.x, positionIn.y - _point1.y);

  else if (slopeAnglePosition2Center > _angleMaxRadians)
    distance = std::hypot(positionIn.x - _point2.x, positionIn.y - _point2.y);

  else
    distance = std::abs(std::hypot(positionIn.x - _center.x, positionIn.y - _center.y) - _radius);

  // std::cout << "Distance from arc: " << distance << '\n';

  return distance;
}

void ArcTrack::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto arcTrackNode {cvFileObjectIn[Types.at(getType())]};

  if (arcTrackNode.empty() || !arcTrackNode.isMap()) {
    throw std::runtime_error(
      "Arc track section is incorrectly formatted or has missing information");
  }

  auto point1XNode {arcTrackNode["point1"]["x_meters"]};
  auto point1YNode {arcTrackNode["point1"]["y_meters"]};
  auto point2XNode {arcTrackNode["point2"]["x_meters"]};
  auto point2YNode {arcTrackNode["point2"]["y_meters"]};
  auto sweepAngleNode {arcTrackNode["sweep_angle_degrees"]};

  if (!point1XNode.isReal() || !point1YNode.isReal()
    || !point2XNode.isReal() || !point2YNode.isReal()
    || !sweepAngleNode.isReal()) {

    throw std::runtime_error(
      "Arc track section is incorrectly formatted or has missing information");
  }

  cv::read(point1XNode, _point1.x, 0.0);
  cv::read(point1YNode, _point1.y, 0.0);
  cv::read(point2XNode, _point2.x, 0.0);
  cv::read(point2YNode, _point2.y, 0.0);

  double sweepAngleDegrees {};
  cv::read(sweepAngleNode, sweepAngleDegrees, 180.0);
  _sweepAngleRadians = sweepAngleDegrees * (M_PI / 180);

  updateParameters();
}

void ArcTrack::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment("");
  cvFileObjectOut.startWriteStruct(Types.at(getType()), cv::FileNode::MAP);

    cvFileObjectOut.startWriteStruct("point1", cv::FileNode::MAP);
      cvFileObjectOut << "x_meters" << _point1.x;
      cvFileObjectOut << "y_meters" << _point1.y;
    cvFileObjectOut.endWriteStruct();

    cvFileObjectOut.startWriteStruct("point2", cv::FileNode::MAP);
      cvFileObjectOut << "x_meters" << _point2.x;
      cvFileObjectOut << "y_meters" << _point2.y;
    cvFileObjectOut.endWriteStruct();

    cvFileObjectOut << "sweep_angle_degrees" << _sweepAngleRadians * (180 / M_PI);

  cvFileObjectOut.endWriteStruct();
}

matplot::line_handle ArcTrack::plot(matplot::axes_handle& axesHandleOut) const
{
  std::vector<double> xPoints {};
  std::vector<double> yPoints {};

  for (double angle {_angleMinRadians}; angle < _angleMaxRadians; angle += M_PI / 180) {
    xPoints.push_back((_center.x + _radius * std::cos(angle)) * 100);
    yPoints.push_back((_center.y + _radius * std::sin(angle)) * 100);
  }

  auto plot {axesHandleOut->plot(xPoints, yPoints)};
  plot->color("green");

  return plot;
}

void ArcTrack::drawOnFrame(
  cv::Mat& frameOut,
  const cv::Affine3d& cameraExtrinsicIn,
  const cv::Mat& cameraIntrinsicCamMatrix,
  const cv::Mat& cameraIntrinsicDistCoeffs)
{
  if (_objectPoints3D.empty())
    return;

  _imagePoints2D.clear();
  _polyLinesPoints.clear();

  cv::projectPoints(
    _objectPoints3D,
    cameraExtrinsicIn.rvec(), cameraExtrinsicIn.translation(),
    cameraIntrinsicCamMatrix, cameraIntrinsicDistCoeffs,
    _imagePoints2D);

  for (auto pointIt {_imagePoints2D.begin()}; pointIt < _imagePoints2D.end(); pointIt++) {
    auto nextPointit {std::next(pointIt)};

    if (nextPointit == _imagePoints2D.end())
      break;

    _polyLinesPoints.push_back({*pointIt, *nextPointit});
  }

  cv::polylines(frameOut, _polyLinesPoints, false, {0, 255, 0});

  // int arcRadiusOnImage = std::abs(_imagePoints2D.at(1).x - _imagePoints2D.at(0).x);

  // std::cout << "Just a quick check: " << '\n'
  //   << "  Center: " << _center << '\n'
  //   << "  Point 1: " << _point1 << '\n'
  //   << "  Point 2: " << _point2 << '\n'
  //   << "  Radius: " << _radius << '\n'
  //   << "  Angle Min: " << _angleMinRadians * (180 / M_PI) << '\n'
  //   << "  Angle Max: " << _angleMaxRadians * (180 / M_PI)<< '\n'
  //   << "  Sweep angle: " << _sweepAngleRadians * (180 / M_PI) << '\n' << '\n';

  // cv::ellipse(
  //   frameOut,
  //   _imagePoints2D.at(0),
  //   {arcRadiusOnImage, arcRadiusOnImage},
  //   _angleMinDegrees,
  //   _angleMinDegrees,
  //   _angleMaxDegrees,
  //   {0, 255, 0});
}

}
