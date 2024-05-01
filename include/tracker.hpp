/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#pragma once

#include "options.hpp"

namespace tracker {

class BoardDetector {

public:
  BoardDetector(
    const options::MarkerDetection& detectionOptions,
    const options::BoardMarkers& boardMarkersOptions,
    bool canEstimatePose = false);

  bool detectBoard(const cv::Mat& frame);
  bool estimateBoardPose();
  bool estimateObjectRelativePose(
    const cv::Vec3d& tVecObjectCamera,
    const cv::Vec3d& rVecObjectCamera);

  void visualize(cv::Mat& frame);

  const std::pair<cv::Vec3d, cv::Vec3d>& getBoardPose() const
  {
    return _poseBoardCamera;
  }

private:
  void reset()
  {
    _detectedMarkerCorners.clear();
    _detectedMarkerIDs.clear();
    _rejectedMarkerCorners.clear();
    _boardDetected = false;
    _boardPoseEstimated = false;
    _tVecBoardCamera = {0.0, 0.0, 0.0};
    _rVecBoardCamera = {0.0, 0.0, 0.0};
    _tVecObjectBoard = {0.0, 0.0, 0.0};
    _rVecObjectBoard = {0.0, 0.0, 0.0};
    _eulerAnglesObjectBoard = {0.0, 0.0, 0.0};
  }

  std::vector<std::vector<cv::Point3f>> getBoardMarkersPoints(
    const options::BoardMarkers& boardMarkersOptions);

  bool hasEnoughBoardIDs();

  bool _canEstimatePose;
  bool _boardDetected;
  bool _boardPoseEstimated;
  float _boardMarkerSide;

  std::pair<cv::Vec3d, cv::Vec3d> _poseBoardCamera;
  cv::Vec3d& _tVecBoardCamera {_poseBoardCamera.first};
  cv::Vec3d& _rVecBoardCamera {_poseBoardCamera.second};

  std::pair<cv::Vec3d, cv::Vec3d> _poseObjectBoard;
  cv::Vec3d& _tVecObjectBoard {_poseObjectBoard.first};
  cv::Vec3d& _rVecObjectBoard {_poseObjectBoard.second};
  cv::Vec3d _eulerAnglesObjectBoard;

  cv::Mat _camMatrix;
  cv::Mat _distortionCoeffs;

  cv::aruco::Board _lineFollowerBoard;
  cv::aruco::ArucoDetector _boardDetector;

  std::vector<int> _detectedMarkerIDs;
  std::vector<std::vector<cv::Point2f>> _detectedMarkerCorners;
  std::vector<std::vector<cv::Point2f>> _rejectedMarkerCorners;
  cv::Mat _boardObjPoints;
  cv::Mat _boardImgPoints;

};

class LineFollowerDetector {

public:
  LineFollowerDetector(
    const options::MarkerDetection& detectionOptions,
    const options::LineFollowerMarker& lineFollowerOptions,
    bool canEstimatePose = false);

  bool detectLineFollower(const cv::Mat& frame);
  bool estimateLineFollowerPose();
  void visualize(cv::Mat& frame);

  std::pair<cv::Vec3d, cv::Vec3d> getLineFollowerPose() const
  {
    return {_lineFollowerRVec, _lineFollowerTVec};
  }
  const cv::Vec3d& getRVec() const {return _lineFollowerRVec;}
  const cv::Vec3d& getTVec() const {return _lineFollowerTVec;}

private:
  void reset()
  {
    _detectedMarkerCornersIterator = _detectedMarkersCorners.begin();
    _detectedMarkersCorners.clear();
    _detectedMarkerIDs.clear();
    _rejectedMarkersCorners.clear();
    _lineFollowerDetected = false;
    _lineFollowerPoseEstimated = false;
    _lineFollowerTVec = {0.0, 0.0, 0.0};
    _lineFollowerRVec = {0.0, 0.0, 0.0};
  }

  bool hasCorrectID();

  bool _canEstimatePose;
  bool _lineFollowerDetected;
  bool _lineFollowerPoseEstimated;
  int _markerID;
  float _markerSide;
  std::vector<std::vector<cv::Point2f>>::iterator _detectedMarkerCornersIterator;

  cv::Vec3d _lineFollowerTVec;
  cv::Vec3d _lineFollowerRVec;

  cv::Mat _camMatrix;
  cv::Mat _distortionCoeffs;

  cv::aruco::ArucoDetector _lineFollowerDetector;

  std::vector<cv::Vec3f> _markerObjPoints;
  // cv::Mat _markerImgPoints;
  std::vector<int> _detectedMarkerIDs;
  std::vector<std::vector<cv::Point2f>> _detectedMarkersCorners;
  std::vector<std::vector<cv::Point2f>> _rejectedMarkersCorners;

};

bool isNonZeroMatrix(const cv::Mat& matrix);

void trackLineFollower(
  const options::MarkerDetection& detectionOptions,
  const options::BoardMarkers& boardMarkersOptions,
  const options::LineFollowerMarker& lineFollowerOptions,
  const std::string& outputFileName = "none");

void calibrateCamera(const options::Calibration& options, const options::CalibrationOutput& output);
}
