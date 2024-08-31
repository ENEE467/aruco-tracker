#pragma once

#include <opencv2/core/affine.hpp>

#include "Tracking.hpp"

namespace tracking {

class BoardDetector {

public:
  BoardDetector(const options::Tracking& optionsIn);

  bool detectBoard(const cv::Mat& frame);
  void visualize(cv::Mat& frameOut);
  void visualize(cv::Mat& frameOut, tracks::Track* trackIn);

  bool estimateFrameBoard_Camera();
  bool estimateFrameLineFollower_Board(const cv::Affine3d& frameLineFollower_Camera);

  const cv::Affine3d& getFrameLineFollower_Board() const {return _frameLineFollower_Board;}
  const cv::Point2d& getPositionXYLineFollower_Board() const {return _positionXYLineFollower_Board;}
  const cv::Affine3d& getFrameBoard_Camera() const {return _frameBoard_Camera;}

private:
  void reset()
  {
    _detectedMarkerCorners.clear();
    _detectedMarkerIDs.clear();
    _rejectedMarkerCorners.clear();
    _boardDetected = false;
    _boardPoseEstimated = false;
    _frameBoard_Camera = cv::Affine3d::Identity();
    _frameLineFollower_Board = cv::Affine3d::Identity();
    _positionXYLineFollower_Board = {0.0, 0.0};
  }

  std::vector<std::vector<cv::Point3f>> getBoardMarkersPoints(
    const options::BoardMarkers& boardMarkersOptions);

  bool hasEnoughBoardIDs();

  bool _canEstimatePose {false};
  bool _boardDetected {false};
  bool _boardPoseEstimated {false};
  float _boardMarkerSide {0};


  /**
   * Variable name convention used here:
   *
   * _frame[of the object]_[relative to this frame]
   * _position[axes][of the object]_[relative to this frame]
  **/
  cv::Affine3d _frameBoard_Camera {cv::Affine3d::Identity()};
  cv::Affine3d _frameLineFollower_Board {cv::Affine3d::Identity()};
  cv::Point2d _positionXYLineFollower_Board {0.0, 0.0};

  // cv::Mat _camMatrix;
  // cv::Mat _distortionCoeffs;
  options::CameraIntrinsic _cameraIntrinsicParams {};

  cv::aruco::Board _trackBoard;
  cv::aruco::ArucoDetector _trackBoardDetector;
  std::vector<cv::Point3d> _trackBoardObjPoints {};
  std::vector<cv::Point2d> _trackBoardImgPoints {};

  std::vector<int> _detectedMarkerIDs {};
  std::vector<std::vector<cv::Point2f>> _detectedMarkerCorners {};
  std::vector<std::vector<cv::Point2f>> _rejectedMarkerCorners {};

};

}
