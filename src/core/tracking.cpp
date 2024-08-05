#include <iostream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <filesystem>
#include <glad/glad.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "core/tracking.hpp"
#include "core/errors.hpp"
#include "core/fileio.hpp"

tracking::BoardDetector::BoardDetector(
  const options::MarkerDetection& detectionOptionsIn,
  const options::BoardMarkers& boardMarkersOptionsIn,
  const options::Track& trackOptionsIn,
  const options::CameraIntrinsic& calibrationParamsIn
)
: _canEstimatePose {calibrationParamsIn.isNonZero()},
  _boardDetected {false},
  _boardPoseEstimated {false},
  _boardMarkerSide {boardMarkersOptionsIn.markerSideMeters},
  _frameBoard_Camera {cv::Affine3d::Identity()},
  _frameLineFollower_Board {cv::Affine3d::Identity()},
  _positionXYLineFollower_Board {0.0, 0.0},
  _camMatrix {calibrationParamsIn.cameraMatrix},
  _distortionCoeffs {calibrationParamsIn.distortionCoefficients},

  _lineFollowerBoard {
    getBoardMarkersPoints(boardMarkersOptionsIn),
    cv::aruco::getPredefinedDictionary(boardMarkersOptionsIn.markerDictionaryID),
    boardMarkersOptionsIn.markerIDs},

  _boardDetector {
    cv::aruco::getPredefinedDictionary(boardMarkersOptionsIn.markerDictionaryID),
    detectionOptionsIn.detectorParameters},

  _trackOptions {trackOptionsIn},
  _trackObjPoints_Board {},
  _trackImgPoints {}
{
  setTrackObjBoardPoints();
}

std::vector<std::vector<cv::Point3f>> tracking::BoardDetector::getBoardMarkersPoints(
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
    translateMarkerObjpoints(0, 0)
  };
}

void tracking::BoardDetector::setTrackObjBoardPoints()
{
  _trackObjPoints_Board.clear();

  switch (_trackOptions.selection) {

  case options::TrackSelection::LINE:
    _trackObjPoints_Board.emplace_back(
      _trackOptions.lineTrack.getPoint1().x,
      _trackOptions.lineTrack.getPoint1().y,
      0);

    _trackObjPoints_Board.emplace_back(
      _trackOptions.lineTrack.getPoint2().x,
      _trackOptions.lineTrack.getPoint2().y,
      0);

    break;

  case options::TrackSelection::ROUND:
    _trackObjPoints_Board.emplace_back(
      _trackOptions.roundTrack.getCenter().x, _trackOptions.roundTrack.getCenter().y, 0);

    _trackObjPoints_Board.emplace_back(
      _trackOptions.roundTrack.getCenter().x + _trackOptions.roundTrack.getMajorAxisLength(),
      _trackOptions.roundTrack.getCenter().y + _trackOptions.roundTrack.getMinorAxisLength(),
      0);

    break;

  default:
    throw Error::INVALID_TRACK_OPTION;

  }
}

bool tracking::BoardDetector::hasEnoughBoardIDs()
{
  int foundMarkersCount {0};

  for (const auto& boardMarkerID: _lineFollowerBoard.getIds()) {
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

  _boardDetector.detectMarkers(
    frame, _detectedMarkerCorners, _detectedMarkerIDs, _rejectedMarkerCorners);

  _boardDetected = hasEnoughBoardIDs();

  return _boardDetected;
}

void tracking::BoardDetector::visualize(cv::Mat& frame)
{
  cv::aruco::drawDetectedMarkers(frame, _detectedMarkerCorners, _detectedMarkerIDs);

  if (!_boardPoseEstimated)
    return;

  drawTrack(frame);

  cv::drawFrameAxes(frame, _camMatrix, _distortionCoeffs, _frameBoard_Camera.rvec(),
    _frameBoard_Camera.translation(), _boardMarkerSide * 1.5f, 2);
}

void tracking::BoardDetector::drawTrack(cv::Mat& frame)
{
  if (_trackObjPoints_Board.empty())
    return;

  cv::projectPoints(
    _trackObjPoints_Board, _frameBoard_Camera.rvec(), _frameBoard_Camera.translation(), _camMatrix,
    _distortionCoeffs, _trackImgPoints);

  float ellipseWidth {};
  float ellipseHeight {};

  switch (_trackOptions.selection) {

  case options::TrackSelection::LINE:
    cv::line(frame, _trackImgPoints.at(0), _trackImgPoints.at(1), {0, 255, 0});
    break;

  case options::TrackSelection::ROUND:
    ellipseWidth = std::abs(_trackImgPoints.at(1).x - _trackImgPoints.at(0).x);
    ellipseHeight = std::abs(_trackImgPoints.at(1).y - _trackImgPoints.at(0).y);

    // std::cout << "Ellipse center: " << _trackImgPoints.at(0) << '\n'
    //           << "Ellipse second track point: " << _trackImgPoints.at(1) << '\n'
    //           << "Ellipse width (half): " << ellipseWidthHalf << '\n'
    //           << "Ellipse height (half): " << ellipseHeightHalf << '\n';

    cv::ellipse(frame, {_trackImgPoints.at(0), {ellipseWidth, ellipseHeight}, 0}, {0, 255, 0});
    break;

  default:
    throw Error::INVALID_TRACK_OPTION;

  }
}

bool tracking::BoardDetector::estimateFrameBoard_Camera()
{
  if (!_boardDetected || !_canEstimatePose) {
    _boardPoseEstimated = false;
    return _boardPoseEstimated;
  }

  _lineFollowerBoard.matchImagePoints(
    _detectedMarkerCorners, _detectedMarkerIDs, _boardObjPoints, _boardImgPoints);

  cv::Vec3d rVecBoard_Camera;
  cv::Vec3d tVecBoard_Camera;

  cv::solvePnP(
    _boardObjPoints, _boardImgPoints, _camMatrix, _distortionCoeffs, rVecBoard_Camera,
    tVecBoard_Camera);

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

tracking::LineFollowerDetector::LineFollowerDetector(
  const options::MarkerDetection& detectionOptions,
  const options::LineFollowerMarker& lineFollowerOptions,
  const options::CameraIntrinsic& calibrationParamsIn
)
: _canEstimatePose {calibrationParamsIn.isNonZero()},
  _lineFollowerDetected {false},
  _lineFollowerPoseEstimated {false},
  _markerID {lineFollowerOptions.markerID},
  _markerSide {lineFollowerOptions.markerSideMeters},
  _iteratorToLineFollowerMarkerCorners {_detectedMarkersCorners.begin()},
  // _lineFollowerTVec {0.0, 0.0, 0.0},
  // _lineFollowerRVec {0.0, 0.0, 0.0},
  _frameLineFollower_Camera {cv::Affine3d::Identity()},
  _camMatrix {calibrationParamsIn.cameraMatrix},
  _distortionCoeffs {calibrationParamsIn.distortionCoefficients},

  _lineFollowerMarkerDetector {
    cv::aruco::getPredefinedDictionary(lineFollowerOptions.markerDictionaryID),
    detectionOptions.detectorParameters},

  _markerObjPoints {
    cv::Vec3f(-_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, -_markerSide / 2.f, 0),
    cv::Vec3f(-_markerSide / 2.f, -_markerSide / 2.f, 0)
  }
{}

bool tracking::LineFollowerDetector::detectLineFollower(const cv::Mat& frame)
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

bool tracking::LineFollowerDetector::estimateFrameLineFollower_Camera()
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
    _markerObjPoints, *_iteratorToLineFollowerMarkerCorners, _camMatrix,
    _distortionCoeffs, rVecLineFollower_Camera, tVecLineFollower_Camera);

  _frameLineFollower_Camera.translation(tVecLineFollower_Camera);
  _frameLineFollower_Camera.rotation(rVecLineFollower_Camera);

  _lineFollowerPoseEstimated = true;

  return _lineFollowerPoseEstimated;
}

void tracking::LineFollowerDetector::visualize(cv::Mat& frame)
{
  if (!_lineFollowerDetected)
    return;

  std::vector<std::vector<cv::Point2f>> lineFollowerMarkerCorners {
    *_iteratorToLineFollowerMarkerCorners};

  cv::aruco::drawDetectedMarkers(frame, lineFollowerMarkerCorners, _detectedMarkerIDs);

  if (!_lineFollowerPoseEstimated)
    return;

  cv::drawFrameAxes(
    frame, _camMatrix, _distortionCoeffs, _frameLineFollower_Camera.rvec(),
    _frameLineFollower_Camera.translation(), _markerSide * 1.5f, 2);
}

bool tracking::LineFollowerDetector::hasCorrectID()
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

void tracking::Output::open(
  const options::Tracking& trackingOptionsIn,
  const std::string& outputParentDirectoryPathIn,
  const std::string& outputNameIn)
{
  _outputDirectoryPath = fileio::createPath(outputParentDirectoryPathIn, "run", outputNameIn, "").str();
  std::filesystem::create_directory(_outputDirectoryPath);
  _outputName = outputNameIn;

  positionsOutput.reset(new fileio::CSVFile(_outputDirectoryPath, "positions", outputNameIn));
  errorsOutput.reset(new fileio::CSVFile(_outputDirectoryPath, "errors", outputNameIn));
  plotsOutput.reset(new plotting::Plotter(trackingOptionsIn.boardMarkers));
  plotsOutput->setReferenceTrack(trackingOptionsIn.track);
}

void tracking::Output::close()
{
  positionsOutput.reset(nullptr);
  errorsOutput.reset(nullptr);

  plotsOutput->savePlots(_outputDirectoryPath, _outputName);
  plotsOutput.reset(nullptr);

  _outputDirectoryPath.clear();
  _outputName.clear();
}

double tracking::calculateTrackingError(
  const cv::Point2d& positionIn,
  const options::Track& trackOptionsIn)
{
  double trackingError {0.0};

  switch (trackOptionsIn.selection) {

  case options::TrackSelection::LINE:
    trackingError = trackOptionsIn.lineTrack.calculatePerpendicularDistance(positionIn);
    break;

  case options::TrackSelection::ROUND:
    trackingError = trackOptionsIn.roundTrack.calculatePerpendicularDistance(positionIn);
    break;

  default:
    throw Error::INVALID_TRACK_OPTION;
    break;

  }

  return trackingError;
}

static GLuint matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter) {
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);

	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);

	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
		magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		// cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}

	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_BGR;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_RED;
	}

	// Create the texture
	glTexImage2D(
    GL_TEXTURE_2D,     // Type of texture
		0,                 // Pyramid level (for mip-mapping) - 0 is the top level
		GL_RGB,            // Internal colour format to convert to
		mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
		mat.rows,          // Image height i.e. 480 for Kinect in standard mode
		0,                 // Border width in pixels (can either be 1 or 0)
		inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
		GL_UNSIGNED_BYTE,  // Image data type
		mat.ptr());        // The actual image data itself

// If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
	if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
		minFilter == GL_LINEAR_MIPMAP_NEAREST ||
		minFilter == GL_NEAREST_MIPMAP_LINEAR ||
		minFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		glGenerateMipmap(GL_TEXTURE_2D);
	}

	return textureID;
}

tracking::Tracker::Tracker(
  const std::string& outputParentDirectoryPathIn,
  const std::string& outputNameIn,
  const options::Tracking& optionsIn)
: _saveOutput {false},
  _options {optionsIn},
  _outputParentDirectory {outputParentDirectoryPathIn},
  _outputName {outputNameIn},

  _startTime {},
  _currentTime {},
  _elapsedTime {},

  _lineFollowerBoardDetector {
    optionsIn.detection, optionsIn.boardMarkers, optionsIn.track, optionsIn.calibrationParams},

  _lineFollowerDetector {
    optionsIn.detection, optionsIn.lineFollowerMarker, optionsIn.calibrationParams}
{
  _inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, optionsIn.detection.frameWidthPixels);
  _inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, optionsIn.detection.frameHeightPixels);
  _inputVideo.set(cv::CAP_PROP_FPS, optionsIn.detection.frameRateFPS);
}

void tracking::Tracker::start()
{
  _inputVideo.open(_options.detection.camID);
  _inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, _options.detection.frameWidthPixels);
  _inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, _options.detection.frameHeightPixels);
  _inputVideo.set(cv::CAP_PROP_FPS, _options.detection.frameRateFPS);
}

void tracking::Tracker::run(unsigned int& imageTextureOut)
{
  if (!_inputVideo.grab())
   return;

  _inputVideo.retrieve(_frame);

  _lineFollowerBoardDetector.detectBoard(_frame);
  _lineFollowerBoardDetector.estimateFrameBoard_Camera();

  _lineFollowerDetector.detectLineFollower(_frame);

  if (_lineFollowerDetector.estimateFrameLineFollower_Camera()) {
    _lineFollowerBoardDetector.estimateFrameLineFollower_Board(
      _lineFollowerDetector.getFrameLineFollower_Camera());
  }

  double trackingError {
    calculateTrackingError(
      _lineFollowerBoardDetector.getPositionXYLineFollower_Board(), _options.track)};

  _lineFollowerBoardDetector.visualize(_frame);
  _lineFollowerDetector.visualize(_frame);

  // cv::imshow("Line Follower tracking", _frame);
  imageTextureOut = matToTexture(_frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);

  char key {static_cast<char>(cv::waitKey(10))};
  // if(key == 27) {
  //   break;
  // }
  // else if (key == 32) {
  if (key == 32) {
    _saveOutput = !_saveOutput;

    if (_saveOutput) {
      _trackingOutput.open(_options, _outputParentDirectory, _outputName);
      _startTime = std::chrono::high_resolution_clock::now();
      std::cout << "Tracking has begun, good luck!" << '\n';
    }
    else if ((!_saveOutput)) {
      _trackingOutput.close();
      std::cout << "Tracking has ended, hit SPACE again to track a new run." << '\n';
    }
  }

  // Output specific stuff only from here.
  if (!_saveOutput)
    return;

  _currentTime = std::chrono::high_resolution_clock::now();
  _elapsedTime = _currentTime - _startTime;

  _trackingOutput.errorsOutput->writeError(trackingError, _elapsedTime.count());

  _trackingOutput.positionsOutput->writePosition(
    _lineFollowerBoardDetector.getPositionXYLineFollower_Board(), _elapsedTime.count());

  _trackingOutput.plotsOutput->saveError(trackingError, _elapsedTime.count());

  _trackingOutput.plotsOutput->savePosition(
    _lineFollowerBoardDetector.getPositionXYLineFollower_Board());
}
