#pragma once

#include <map>

#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/affine.hpp>

#include <matplot/matplot.h>

namespace tracks {

enum class Type {

  NONE = 0,
  LINE,
  ARC,
  ROUND,
  POLYGON,
  PILL

};

const std::map<Type, std::string> Types {
  {Type::LINE, "line_track"},
  {Type::ARC, "arc_track"},
  {Type::ROUND, "round_track"},
  {Type::POLYGON, "polygon_track"},
  {Type::PILL, "pill_track"}
};

class Track {

public:
  Track() {}

  virtual const Type getType() const {return Type::NONE;}

  virtual double calculatePerpendicularDistance(const cv::Point2d& positionIn) const
  {
    throw std::runtime_error("Specific type of track isn't assigned.");
  }

  virtual void plot(matplot::axes_handle& axesHandleOut) const
  {
    throw std::runtime_error("Specific type of track isn't assigned.");
  }

  virtual void drawOnFrame(
    cv::Mat& frameOut,
    const cv::Affine3d& cameraExtrinsicIn,
    const cv::Mat& cameraIntrinsicCamMatrix,
    const cv::Mat& cameraIntrinsicDistCoeffs)
  {
    throw std::runtime_error("Specific type of track isn't assigned.");
  }

  virtual void readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
  {
    throw std::runtime_error("Specific type of track isn't assigned.");
  }

  virtual void writeToConfigFile(cv::FileStorage& cvFileObjectOut)
  {
    throw std::runtime_error("Specific type of track isn't assigned.");
  }

protected:
  // Used for drawing method
  std::vector<cv::Point3d> _objectPoints3D {};
  std::vector<cv::Point2d> _imagePoints2D {};

};

}
