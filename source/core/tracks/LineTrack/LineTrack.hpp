#pragma once

#include "Track.hpp"

namespace tracks {

class LineTrack : public Track {

public:
  LineTrack() : Track() {}
  LineTrack(const cv::FileStorage& cvFileObjectIn);
  LineTrack(const cv::Point2d& point1In, const cv::Point2d& point2In);

  void setPoints(const cv::Point2d& point1In, const cv::Point2d& point2In);

  const Type getType() const override {return Type::LINE;}
  const cv::Point2d& getPoint1() const {return _point1;}
  const cv::Point2d& getPoint2() const {return _point2;}
  const double getLength() const {return _length;}

  void plot(matplot::axes_handle& axesHandleOut) const override;
  void drawOnFrame(
    cv::Mat& frameOut,
    const cv::Affine3d& cameraExtrinsicIn,
    const cv::Mat& cameraIntrinsicCamMatrix,
    const cv::Mat& cameraIntrinsicDistCoeffs) override;

  double calculatePerpendicularDistance(const cv::Point2d& positionIn) const override;
  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn) override;
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut) override;

private:
  void updateLength();

  cv::Point2d _point1 {0, 0};
  cv::Point2d _point2 {0, 0};
  double _length {0};
  double _lengthInv {0};

};

}
