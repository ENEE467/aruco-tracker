#include <iostream>

#include "PillTrack.hpp"
#include "LineTrack/LineTrack.hpp"
#include "ArcTrack/ArcTrack.hpp"

namespace tracks {

PillTrack::PillTrack(const cv::Point2d& centerIn, double widthIn, double heightIn)
: Track(),
  _center {centerIn},
  _width {std::abs(widthIn)},
  _height {std::abs(heightIn)},
  _lineLength {widthIn - heightIn}
{
  if (_lineLength <= 0) {
    throw std::runtime_error(
      "Width must always be greater than height when creating a pill track object.");
  }

  setTracks();
}

void PillTrack::setTracks()
{
  cv::Point2d point1 {_center.x + _lineLength / 2, _center.y - _height / 2};
  cv::Point2d point2 {_center.x + _lineLength / 2, _center.y + _height / 2};
  cv::Point2d point3 {_center.x - _lineLength / 2, _center.y + _height / 2};
  cv::Point2d point4 {_center.x - _lineLength / 2, _center.y - _height / 2};

  _tracks.at(0).reset(new ArcTrack(point1, point2, 180));
  _tracks.at(1).reset(new LineTrack(point2, point3));
  _tracks.at(2).reset(new ArcTrack(point3, point4, 180));
  _tracks.at(3).reset(new LineTrack(point4, point1));
}

double PillTrack::calculatePerpendicularDistance(const cv::Point2d& positionIn) const
{
  double distance {INFINITY};
  double newDistance {};

  for (const auto& track: _tracks) {
    newDistance = track->calculatePerpendicularDistance(positionIn);

    if (newDistance >= distance)
      continue;

    distance = newDistance;
  }

  return distance;
}

void PillTrack::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
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

void PillTrack::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto pillTrackNode {cvFileObjectIn[Types.at(getType())]};

  if (pillTrackNode.empty() || !pillTrackNode.isMap()) {
    throw std::runtime_error(
      "Pill track section is incorrectly formatted or has missing information.");
  }

  auto centerXNode {pillTrackNode["center"]["x_meters"]};
  auto centerYNode {pillTrackNode["center"]["y_meters"]};
  auto widthNode {pillTrackNode["width_meters"]};
  auto heightNode {pillTrackNode["height_meters"]};

  if (!centerXNode.isReal() || !centerXNode.isReal()
    || !widthNode.isReal() || !heightNode.isReal()) {

    throw std::runtime_error(
      "Pill track section is incorrectly formatted or has missing information.");
  }

  cv::read(centerXNode, _center.x, 0.0);
  cv::read(centerYNode, _center.y, 0.0);
  cv::read(widthNode, _width, 0.0);
  cv::read(heightNode, _height, 0.0);

  _lineLength = _width - _height;

  if (_lineLength <= 0) {
    throw std::runtime_error(
      "Width must always be greater than height when creating a pill track object.");
  }

  setTracks();
}

matplot::line_handle PillTrack::plot(matplot::axes_handle& axesHandleOut) const
{
  matplot::line_handle lastTrackPlot {};
  axesHandleOut->hold(matplot::on);

  for (const auto& track: _tracks)
    lastTrackPlot = track->plot(axesHandleOut);

  axesHandleOut->hold(matplot::off);

  return lastTrackPlot;
}

void PillTrack::drawOnFrame(
    cv::Mat& frameOut,
    const cv::Affine3d& cameraExtrinsicIn,
    const cv::Mat& cameraIntrinsicCamMatrix,
    const cv::Mat& cameraIntrinsicDistCoeffs)
{
  for (const auto& track: _tracks) {
    track->drawOnFrame(
      frameOut, cameraExtrinsicIn, cameraIntrinsicCamMatrix, cameraIntrinsicDistCoeffs);
  }
}

}
