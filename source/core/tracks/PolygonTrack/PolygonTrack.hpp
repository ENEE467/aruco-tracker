#pragma once

#include "LineTrack/LineTrack.hpp"

namespace tracks {

class PolygonTrack : public Track {

public:
  PolygonTrack() : Track() {}
  PolygonTrack(const cv::FileStorage& cvFileObjectIn);
  PolygonTrack(const cv::Point2d& centerIn, int sidesIn, double widthIn, double heightIn);

  void drawOnFrame(
    cv::Mat& frameOut,
    const cv::Affine3d& cameraExtrinsicIn,
    const cv::Mat& cameraIntrinsicCamMatrix,
    const cv::Mat& cameraIntrinsicDistCoeffs) override;

  const Type getType() const override {return Type::POLYGON;}
  const std::vector<cv::Point2d>& getVertices() const {return _vertices;}

  void plot(matplot::axes_handle& axesHandleOut) const override;

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const override;

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn) override;
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut) override;

private:
  void generateSidesAndVertices();

  cv::Point2d _center {0, 0};
  double _width {0};
  double _height {0};
  int _sidesQuantity {0};

  std::vector<LineTrack> _sides {};
  std::vector<std::vector<cv::Point2i>> _polyLinesPoints {};
};

}
