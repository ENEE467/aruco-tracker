#pragma once

#include "Track.hpp"

namespace tracks {

class ArcTrack : public Track {

public:
  ArcTrack() : Track() {}
  ArcTrack(const cv::FileStorage& cvFileObjectIn);
  ArcTrack(const cv::Point2d& point1In, const cv::Point2d& point2In, double sweepDegrees = 90);

  const Type getType() const override {return Type::ARC;}

  matplot::line_handle plot(matplot::axes_handle& axesHandleOut) const override;
  void drawOnFrame(
    cv::Mat& frameOut,
    const cv::Affine3d& cameraExtrinsicIn,
    const cv::Mat& cameraIntrinsicCamMatrix,
    const cv::Mat& cameraIntrinsicDistCoeffs) override;

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const override;

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn) override;
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut) override;

private:
  void updateParameters();

  cv::Point2d _center {0, 0};
  cv::Point2d _point1 {0, 0};
  cv::Point2d _point2 {0, 0};

  double _radius {};
  double _sweepAngleRadians {};
  // double _angleMinDegrees {};
  // double _angleMaxDegrees {};
  double _angleMinRadians {};
  double _angleMaxRadians {};
  std::vector<std::vector<cv::Point2i>> _polyLinesPoints {};

};

}
