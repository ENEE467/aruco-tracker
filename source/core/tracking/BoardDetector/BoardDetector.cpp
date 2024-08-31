#include <opencv2/calib3d.hpp>

#include "BoardDetector.hpp"


namespace tracking {

BoardDetector::BoardDetector(const options::Tracking& optionsIn)
: _canEstimatePose {optionsIn.intrinsicParams.isNonZero()},
  _boardMarkerSide {optionsIn.boardMarkers.markerSideMeters},
  _cameraIntrinsicParams {optionsIn.intrinsicParams},

  _trackBoard {
    getBoardMarkersPoints(optionsIn.boardMarkers),
    cv::aruco::getPredefinedDictionary(optionsIn.boardMarkers.markerDictionaryID),
    optionsIn.boardMarkers.markerIDs},

  _trackBoardDetector {
    cv::aruco::getPredefinedDictionary(optionsIn.boardMarkers.markerDictionaryID),
    optionsIn.detection.detectorParameters}
{}

std::vector<std::vector<cv::Point3f>> BoardDetector::getBoardMarkersPoints(
  const options::BoardMarkers& boardMarkersOptions)
{
  std::vector<cv::Point3f> markerObjPoints {
    cv::Point3f(
      -boardMarkersOptions.markerSideMeters/2.f,
      boardMarkersOptions.markerSideMeters/2.f,
      0),

    cv::Point3f(
      boardMarkersOptions.markerSideMeters/2.f,
      boardMarkersOptions.markerSideMeters/2.f,
      0),

    cv::Point3f(
      boardMarkersOptions.markerSideMeters/2.f,
      -boardMarkersOptions.markerSideMeters/2.f,
      0),

    cv::Point3f(
      -boardMarkersOptions.markerSideMeters/2.f,
      -boardMarkersOptions.markerSideMeters/2.f,
      0)
  };

  auto translateMarkerObjpoints =
    [markerObjPoints] (float xDisplacement, float yDisplacement)
  {
    std::vector<cv::Point3f> translatedPoints {};

    for (const auto& markerObjPoint: markerObjPoints) {
      translatedPoints.push_back(markerObjPoint + cv::Point3f(xDisplacement, yDisplacement, 0));
    }

    return translatedPoints;
  };

  return {
    translateMarkerObjpoints(0, boardMarkersOptions.markerSeperationMetersY),

    translateMarkerObjpoints(
      boardMarkersOptions.markerSeperationMetersX,
      boardMarkersOptions.markerSeperationMetersY),

    translateMarkerObjpoints(boardMarkersOptions.markerSeperationMetersX, 0),
    translateMarkerObjpoints(0, 0)};
}

bool tracking::BoardDetector::hasEnoughBoardIDs()
{
  int foundMarkersCount {0};

  for (const auto& boardMarkerID: _trackBoard.getIds()) {
    auto foundMarkerID {
      std::find(_detectedMarkerIDs.begin(), _detectedMarkerIDs.end(), boardMarkerID)};

    if (foundMarkerID != _detectedMarkerIDs.end())
      foundMarkersCount++;
  }

  if (foundMarkersCount < 3)
    return false;

  return true;
}

bool tracking::BoardDetector::detectBoard(const cv::Mat& frame)
{
  reset();

  _trackBoardDetector.detectMarkers(
    frame, _detectedMarkerCorners, _detectedMarkerIDs, _rejectedMarkerCorners);

  _boardDetected = hasEnoughBoardIDs();

  return _boardDetected;
}

void tracking::BoardDetector::visualize(cv::Mat& frameOut)
{
  cv::aruco::drawDetectedMarkers(frameOut, _detectedMarkerCorners, _detectedMarkerIDs);

  if (!_boardPoseEstimated)
    return;

  cv::drawFrameAxes(
    frameOut,
    _cameraIntrinsicParams.cameraMatrix, _cameraIntrinsicParams.distortionCoefficients,
    _frameBoard_Camera.rvec(), _frameBoard_Camera.translation(),
    _boardMarkerSide * 1.5f, 2);
}

void tracking::BoardDetector::visualize(cv::Mat& frameOut, tracks::Track* trackIn)
{
  visualize(frameOut);

  if (!_boardPoseEstimated || trackIn->getType() == tracks::Type::NONE)
    return;

  trackIn->drawOnFrame(
    frameOut,
    _frameBoard_Camera,
    _cameraIntrinsicParams.cameraMatrix, _cameraIntrinsicParams.distortionCoefficients);
}

bool tracking::BoardDetector::estimateFrameBoard_Camera()
{
  if (!_boardDetected || !_canEstimatePose) {
    _boardPoseEstimated = false;
    return _boardPoseEstimated;
  }

  _trackBoard.matchImagePoints(
    _detectedMarkerCorners, _detectedMarkerIDs, _trackBoardObjPoints, _trackBoardImgPoints);

  cv::Vec3d rVecBoard_Camera;
  cv::Vec3d tVecBoard_Camera;

  cv::solvePnP(
    _trackBoardObjPoints, _trackBoardImgPoints,
    _cameraIntrinsicParams.cameraMatrix, _cameraIntrinsicParams.distortionCoefficients,
    rVecBoard_Camera, tVecBoard_Camera);

  _frameBoard_Camera.translation(tVecBoard_Camera);
  _frameBoard_Camera.rotation(rVecBoard_Camera);

  _boardPoseEstimated = true;

  return _boardPoseEstimated;
}

bool tracking::BoardDetector::estimateFrameLineFollower_Board(
  const cv::Affine3d& frameLineFollower_Camera)
{
  if (!_boardPoseEstimated)
    return false;

  // _frameLineFollower_Board = _frameBoard_Camera * frameLineFollower_Camera.inv();
  _frameLineFollower_Board = _frameBoard_Camera.inv() * frameLineFollower_Camera;

  auto _positionXYZ {_frameLineFollower_Board.translation()};

  _positionXYLineFollower_Board.x = _positionXYZ[0];
  _positionXYLineFollower_Board.y = _positionXYZ[1];

  // Eigen::Matrix3d rotationMatrix;
  // cv::cv2eigen(_frameLineFollower_Board.rotation(), rotationMatrix);

  // Eigen::Vector3d eulerAngles {rotationMatrix.eulerAngles(0, 1, 2)};
  // _eulerAnglesLineFollower_Board = {eulerAngles(0), eulerAngles(1), eulerAngles(2)};

  return true;
}

}
