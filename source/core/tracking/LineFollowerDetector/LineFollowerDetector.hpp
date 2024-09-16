#pragma once

#include <opencv2/core/affine.hpp>

#include "Tracking.hpp"

namespace tracking {

class LineFollowerDetector {

public:
  LineFollowerDetector(const options::Tracking& optionsIn);

  bool detectLineFollower(const cv::Mat& frame);
  bool estimateFrameLineFollower_Camera();
  void visualize(cv::Mat& frame);

  const cv::Affine3d& getFrameLineFollower_Camera() const
  {
    return _frameLineFollower_Camera;
  }

private:
  void reset()
  {
    _iteratorToLineFollowerMarkerCorners = _detectedMarkersCorners.begin();
    _detectedMarkersCorners.clear();
    _detectedMarkerIDs.clear();
    _rejectedMarkersCorners.clear();
    _lineFollowerDetected = false;
    _lineFollowerPoseEstimated = false;
    _frameLineFollower_Camera = cv::Affine3d::Identity();
  }

  bool hasCorrectID();

  bool _canEstimatePose {false};
  bool _lineFollowerDetected {false};
  bool _lineFollowerPoseEstimated {false};
  int _markerID {};
  float _markerSide {};
  std::vector<std::vector<cv::Point2f>>::iterator _iteratorToLineFollowerMarkerCorners {
    _detectedMarkersCorners.begin()};

  cv::Affine3d _frameLineFollower_Camera {cv::Affine3d::Identity()};

  options::CameraIntrinsic _cameraIntrinsicParams {};

  cv::aruco::ArucoDetector _lineFollowerMarkerDetector;
  std::vector<cv::Vec3f> _markerObjPoints {};
  std::vector<int> _detectedMarkerIDs {};
  std::vector<std::vector<cv::Point2f>> _detectedMarkersCorners {};
  std::vector<std::vector<cv::Point2f>> _rejectedMarkersCorners {};

};

}
