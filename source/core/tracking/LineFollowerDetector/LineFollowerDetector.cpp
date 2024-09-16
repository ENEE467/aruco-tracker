#include <opencv2/calib3d.hpp>

#include "LineFollowerDetector.hpp"

namespace tracking {

LineFollowerDetector::LineFollowerDetector(const options::Tracking& optionsIn)
: _canEstimatePose {optionsIn.intrinsicParams.isNonZero()},
  _markerID {optionsIn.lineFollowerMarker.markerID},
  _markerSide {optionsIn.lineFollowerMarker.markerSideMeters},
  _cameraIntrinsicParams {optionsIn.intrinsicParams},

  _lineFollowerMarkerDetector {
    cv::aruco::getPredefinedDictionary(optionsIn.lineFollowerMarker.markerDictionaryID),
    optionsIn.detection.detectorParameters},

  _markerObjPoints {
    cv::Vec3f(-_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, -_markerSide / 2.f, 0),
    cv::Vec3f(-_markerSide / 2.f, -_markerSide / 2.f, 0)
  }
{}

bool LineFollowerDetector::detectLineFollower(const cv::Mat& frame)
{
  reset();

  _lineFollowerMarkerDetector.detectMarkers(
    frame, _detectedMarkersCorners, _detectedMarkerIDs, _rejectedMarkersCorners);

  _lineFollowerDetected = hasCorrectID();

  // std::cout << "Detected IDs: ";
  // for (const auto& markerID: _detectedMarkerIDs)
  //   std::cout << markerID << " ";
  // std::cout << '\n';

  // std::cout << "Line follower " << _markerID << " detected: " << _lineFollowerDetected << '\n';

  return _lineFollowerDetected;
}

bool LineFollowerDetector::estimateFrameLineFollower_Camera()
{
  if (!_lineFollowerDetected || !_canEstimatePose) {
    _lineFollowerPoseEstimated = false;

    return _lineFollowerPoseEstimated;
  }

  // std::cout << "Marker corners: ";
  // for (const auto& corner: *_detectedMarkerCornersIterator)
  //   std::cout << " " << corner.x << ", " << corner.y << " ";
  // std::cout << '\n';

  cv::Vec3d rVecLineFollower_Camera;
  cv::Vec3d tVecLineFollower_Camera;

  cv::solvePnP(
    _markerObjPoints, *_iteratorToLineFollowerMarkerCorners,
    _cameraIntrinsicParams.cameraMatrix, _cameraIntrinsicParams.distortionCoefficients,
    rVecLineFollower_Camera, tVecLineFollower_Camera);

  _frameLineFollower_Camera.translation(tVecLineFollower_Camera);
  _frameLineFollower_Camera.rotation(rVecLineFollower_Camera);

  _lineFollowerPoseEstimated = true;

  return _lineFollowerPoseEstimated;
}

void LineFollowerDetector::visualize(cv::Mat& frameOut)
{
  if (!_lineFollowerDetected)
    return;

  std::vector<std::vector<cv::Point2f>> lineFollowerMarkerCorners {
    *_iteratorToLineFollowerMarkerCorners};

  cv::aruco::drawDetectedMarkers(frameOut, lineFollowerMarkerCorners, _detectedMarkerIDs);

  if (!_lineFollowerPoseEstimated)
    return;

  cv::drawFrameAxes(
    frameOut,
    _cameraIntrinsicParams.cameraMatrix, _cameraIntrinsicParams.distortionCoefficients,
    _frameLineFollower_Camera.rvec(), _frameLineFollower_Camera.translation(),
    _markerSide * 1.5f, 2);
}

bool LineFollowerDetector::hasCorrectID()
{
  auto foundID {std::find(_detectedMarkerIDs.begin(), _detectedMarkerIDs.end(), _markerID)};

  if (foundID == _detectedMarkerIDs.end()) {
    _lineFollowerDetected = false;
    return _lineFollowerDetected;
  }

  auto index = foundID - _detectedMarkerIDs.begin();
  // std::cout << "Index: " << index << '\n';
  _iteratorToLineFollowerMarkerCorners = _detectedMarkersCorners.begin() + index;

  _lineFollowerDetected = true;
  return _lineFollowerDetected;
}

}
