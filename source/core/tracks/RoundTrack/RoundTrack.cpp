#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "RoundTrack.hpp"

namespace tracks {

RoundTrack::RoundTrack(const cv::Point2d& centerIn, double widthIn, double heightIn)
: Track(),
  _center {centerIn},
  _width {widthIn},
  _height {heightIn}
{
  updateParameters();

  _objectPoints3D.reserve(16);
  _imagePoints2D.reserve(16);

  for (double theta {0}; theta <= M_PI * 2; theta += (2 * M_PI) / 16) {
    _objectPoints3D.emplace_back(
      _center.x + _a * std::cos(theta),
      _center.y + _b * std::sin(theta),
      0);
  }
}

RoundTrack::RoundTrack(const cv::FileStorage& cvFileObjectIn)
: Track()
{
  readFromConfigFile(cvFileObjectIn);

  _objectPoints3D.reserve(16);
  _imagePoints2D.reserve(16);

  for (double theta {0}; theta <= M_PI * 2; theta += (2 * M_PI) / 16) {
    _objectPoints3D.emplace_back(
      _center.x + _a * std::cos(theta),
      _center.y + _b * std::sin(theta),
      0);
  }
}

void RoundTrack::setParameters(const cv::Point2d& centerIn, double widthIn, double heightIn)
{
  _center = centerIn;
  _width = widthIn;
  _height = heightIn;

  updateParameters();
}

double RoundTrack::calculatePerpendicularDistance(const cv::Point2d& positionIn) const
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

void RoundTrack::updateParameters()
{
  _a = _width / 2;
  _b = _height / 2;
  _aInv = _a ? 1 / _a : 0;
  _bInv = _b ? 1 / _b : 0;
  _evoluteXCalcMidPart = (_a*_a - _b*_b) * _aInv;
  _evoluteYCalcMidPart = (_b*_b - _a*_a) * _bInv;
}

void RoundTrack::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto roundTrackNode {cvFileObjectIn[Types.at(getType())]};

  if (roundTrackNode.empty() || !roundTrackNode.isMap()) {
    throw std::runtime_error(
      "Round track section is incorrectly formatted or has missing information");
  }

  auto centerXNode {roundTrackNode["center"]["x_meters"]};
  auto centerYNode {roundTrackNode["center"]["y_meters"]};
  auto widthNode {roundTrackNode["width_meters"]};
  auto heightNode {roundTrackNode["height_meters"]};

  if (!centerXNode.isReal() || !centerYNode.isReal()
    || !widthNode.isReal() || !heightNode.isReal()) {

    throw std::runtime_error(
      "Round track section is incorrectly formatted or has missing information");
  }

  cv::read(centerXNode, _center.x, 0.0);
  cv::read(centerYNode, _center.y, 0.0);
  cv::read(widthNode, _width, 0.0);
  cv::read(heightNode, _height, 0.0);

  updateParameters();
}

void RoundTrack::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment("");
  cvFileObjectOut.startWriteStruct(Types.at(getType()), cv::FileNode::MAP);

    cvFileObjectOut.startWriteStruct("center", cv::FileNode::MAP);
      cvFileObjectOut << "x_meters" << _center.x;
      cvFileObjectOut << "y_meters" << _center.y;
    cvFileObjectOut.endWriteStruct();

    cvFileObjectOut << "width_meters" << _width;
    cvFileObjectOut << "height_meters" << _height;

  cvFileObjectOut.endWriteStruct();
}

matplot::line_handle RoundTrack::plot(matplot::axes_handle& axesHandleOut) const
{
  auto plot = axesHandleOut->ellipse(
    _center.x * 100 - _a * 100, _center.y * 100 - _b * 100,
    _width * 100, _height * 100);

  plot->color("green");

  return plot;
}

void RoundTrack::drawOnFrame(
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

  cv::polylines(frameOut, _polyLinesPoints, true, {0, 255, 0});
}

}
