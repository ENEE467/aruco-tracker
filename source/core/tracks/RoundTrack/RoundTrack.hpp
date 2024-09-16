#pragma once

#include "Track.hpp"

namespace tracks {

class RoundTrack : public Track {

public:
  RoundTrack() : Track() {}
  RoundTrack(const cv::FileStorage& cvFileObjectIn) : Track() {readFromConfigFile(cvFileObjectIn);}
  RoundTrack(const cv::Point2d& centerIn, double widthIn, double heightIn);

  void setParameters(const cv::Point2d& centerIn, double widthIn, double heightIn);

  const Type getType() const override {return Type::ROUND;}

  const cv::Point2d& getCenter() const {return _center;}
  const double getWidth() const {return _width;}
  const double getHeight() const {return _height;}
  const double getWidthHalf() const {return _a;}
  const double getHeightHalf() const {return _b;}

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
  double _width {0};
  double _height {0};

  double _a {0};
  double _b {0};
  double _aInv {0};
  double _bInv {0};
  double _evoluteXCalcMidPart {0};
  double _evoluteYCalcMidPart {0};

};

}
