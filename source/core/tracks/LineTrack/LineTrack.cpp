#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "LineTrack.hpp"

namespace tracks {

LineTrack::LineTrack(const cv::Point2d& point1In, const cv::Point2d& point2In)
: Track(),
  _point1 {point1In},
  _point2 {point2In}
{
  updateLength();

  _objectPoints3D.reserve(2);
  _imagePoints2D.reserve(2);
  _objectPoints3D.emplace_back(_point1.x, _point1.y, 0);
  _objectPoints3D.emplace_back(_point2.x, _point2.y, 0);
}

LineTrack::LineTrack(const cv::FileStorage& cvFileObjectIn)
: Track()
{
  readFromConfigFile(cvFileObjectIn);

  _objectPoints3D.reserve(2);
  _objectPoints3D.emplace_back(_point1.x, _point1.y, 0);
  _objectPoints3D.emplace_back(_point2.x, _point2.y, 0);
}

void LineTrack::setPoints(const cv::Point2d& point1In, const cv::Point2d& point2In)
{
  _point1 = point1In;
  _point2 = point2In;

  updateLength();
}

double LineTrack::calculatePerpendicularDistance(const cv::Point2d& positionIn) const
{
  double distanceToLine {0};

  if (_length <= 0)
    return distanceToLine;

  double yDiff {_point2.y - _point1.y};
  double xDiff {_point2.x - _point1.x};
  double c {_point2.x * _point1.y - _point2.y * _point1.x};

  distanceToLine = std::abs(yDiff * positionIn.x - xDiff * positionIn.y + c) * _lengthInv;

  return distanceToLine;
}

void LineTrack::updateLength()
{
  _length = std::hypot(_point2.x - _point1.x, _point2.y - _point1.y);
  _lengthInv = _length ? 1 / _length : 0;
}

void LineTrack::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto lineTrackNode {cvFileObjectIn[Types.at(getType())]};

  if (lineTrackNode.empty() || !lineTrackNode.isMap()) {
    throw std::runtime_error(
      "Line track section is incorrectly formatted or has missing information.");
  }

  auto point1XNode {lineTrackNode["point1"]["x_meters"]};
  auto point1YNode {lineTrackNode["point1"]["y_meters"]};
  auto point2XNode {lineTrackNode["point2"]["x_meters"]};
  auto point2YNode {lineTrackNode["point2"]["y_meters"]};

  if (!point1XNode.isReal() || !point1YNode.isReal()
    || !point2XNode.isReal() || !point2YNode.isReal()) {

    throw std::runtime_error(
      "Line track section is incorrectly formatted or has missing information.");
  }

  cv::read(point1XNode, _point1.x, 0.0);
  cv::read(point1YNode, _point1.y, 0.0);
  cv::read(point2XNode, _point2.x, 0.0);
  cv::read(point2YNode, _point2.y, 0.0);

  updateLength();
}

void LineTrack::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
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

  cvFileObjectOut.endWriteStruct();
}

matplot::line_handle LineTrack::plot(matplot::axes_handle& axesHandleOut) const
{
  auto plot {axesHandleOut->line(_point1.x * 100, _point1.y * 100, _point2.x * 100, _point2.y * 100)};
  plot->color("green");

  return plot;
}

void LineTrack::drawOnFrame(
  cv::Mat& frameOut,
  const cv::Affine3d& cameraExtrinsicIn,
  const cv::Mat& cameraIntrinsicCamMatrix,
  const cv::Mat& cameraIntrinsicDistCoeffs)
{
  if (_objectPoints3D.empty())
    return;

  _imagePoints2D.clear();

  cv::projectPoints(
    _objectPoints3D,
    cameraExtrinsicIn.rvec(), cameraExtrinsicIn.translation(),
    cameraIntrinsicCamMatrix, cameraIntrinsicDistCoeffs,
    _imagePoints2D);

  cv::line(frameOut, _imagePoints2D.at(0), _imagePoints2D.at(1), {0, 255, 0});
}

}
