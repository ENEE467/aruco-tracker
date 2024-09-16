#pragma once

#include "Track.hpp"

namespace tracks {

class PillTrack : public Track {

public:
  PillTrack() : Track() {}
  PillTrack(const cv::Point2d& centerIn, double widthIn, double heightIn);

  PillTrack(const cv::FileStorage& cvFileObjectIn)
  : Track()
  {
    readFromConfigFile(cvFileObjectIn);
  }

  const Type getType() const override {return Type::PILL;}

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
  void setTracks();

  cv::Point2d _center {0, 0};
  double _width {};
  double _height {};
  double _lineLength {};

  std::array<std::unique_ptr<Track>, 4> _tracks {};

};

}
