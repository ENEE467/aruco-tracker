#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>

#include "errors.hpp"
#include "fileio.hpp"
#include "tracker.hpp"

tracker::BoardDetector::BoardDetector(
  const options::MarkerDetection& detectionOptions,
  const options::BoardMarkers& boardMarkersOptions,
  bool canEstimatePose)
: _canEstimatePose {canEstimatePose},
  _boardDetected {false},
  _boardPoseEstimated {false},
  _boardMarkerSide {boardMarkersOptions.markerSideMeters},
  _poseBoardCamera {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  _poseObjectBoard {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
  _eulerAnglesObjectBoard {0.0, 0.0, 0.0},
  _camMatrix {detectionOptions.camMatrix},
  _distortionCoeffs {detectionOptions.distCoeffs},

  _lineFollowerBoard {
    getBoardMarkersPoints(boardMarkersOptions),
    cv::aruco::getPredefinedDictionary(boardMarkersOptions.markerDictionaryID),
    boardMarkersOptions.markerIDs},

  _boardDetector {
    cv::aruco::getPredefinedDictionary(boardMarkersOptions.markerDictionaryID),
    detectionOptions.detectorParameters}
{}

std::vector<std::vector<cv::Point3f>> tracker::BoardDetector::getBoardMarkersPoints(
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
      translatedPoints.push_back(
        markerObjPoint + cv::Point3f(xDisplacement, yDisplacement, 0));
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

bool tracker::BoardDetector::hasEnoughBoardIDs()
{
  int foundMarkersCount {0};

  for (const auto& boardMarkerID: _lineFollowerBoard.getIds()) {
    auto foundMarkerID {
      std::find(
        _detectedMarkerIDs.begin(), _detectedMarkerIDs.end(), boardMarkerID)};

    if (foundMarkerID != _detectedMarkerIDs.end())
      foundMarkersCount++;
  }

  if (foundMarkersCount < 3)
    return false;

  return true;
}

bool tracker::BoardDetector::detectBoard(const cv::Mat& frame)
{
  reset();

  _boardDetector.detectMarkers(
    frame, _detectedMarkerCorners, _detectedMarkerIDs, _rejectedMarkerCorners);

  _boardDetected = hasEnoughBoardIDs();

  return _boardDetected;
}

void tracker::BoardDetector::visualize(cv::Mat& frame)
{
  cv::aruco::drawDetectedMarkers(frame, _detectedMarkerCorners, _detectedMarkerIDs);

  if (!_boardPoseEstimated)
    return;

  cv::drawFrameAxes(
    frame, _camMatrix, _distortionCoeffs, _boardRVec, _boardTVec,
    _boardMarkerSide * 1.5f, 2);
}

bool tracker::BoardDetector::estimateBoardPose()
{
  if (!_boardDetected || !_canEstimatePose) {
    _boardPoseEstimated = false;
    return _boardPoseEstimated;
  }

  _lineFollowerBoard.matchImagePoints(
    _detectedMarkerCorners, _detectedMarkerIDs, _boardObjPoints,
    _boardImgPoints);

  cv::solvePnP(
    _boardObjPoints, _boardImgPoints, _camMatrix, _distortionCoeffs, _boardRVec,
    _boardTVec);

  _boardPoseEstimated = true;

  return _boardPoseEstimated;
}

bool tracker::isNonZeroMatrix(const cv::Mat& matrix)
{
  cv::Mat zeroMatrix {cv::Mat::zeros(matrix.rows, matrix.cols, matrix.type())};
  auto comparisonMatrix {matrix != zeroMatrix};

  return cv::countNonZero(comparisonMatrix);
}

tracker::LineFollowerDetector::LineFollowerDetector(
  const options::MarkerDetection& detectionOptions,
  const options::LineFollowerMarker& lineFollowerOptions,
  bool canEstimatePose
)
: _canEstimatePose {canEstimatePose},
  _lineFollowerDetected {false},
  _lineFollowerPoseEstimated {false},
  _markerID {lineFollowerOptions.markerID},
  _markerSide {lineFollowerOptions.markerSideMeters},
  _detectedMarkerCornersIterator {_detectedMarkersCorners.begin()},
  _lineFollowerTVec {0.0, 0.0, 0.0},
  _lineFollowerRVec {0.0, 0.0, 0.0},
  _camMatrix {detectionOptions.camMatrix},
  _distortionCoeffs {detectionOptions.distCoeffs},

  _lineFollowerDetector {
    cv::aruco::getPredefinedDictionary(lineFollowerOptions.markerDictionaryID),
    detectionOptions.detectorParameters},

  _markerObjPoints {
    cv::Vec3f(-_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, _markerSide / 2.f, 0),
    cv::Vec3f(_markerSide / 2.f, -_markerSide / 2.f, 0),
    cv::Vec3f(-_markerSide / 2.f, -_markerSide / 2.f, 0)
  }
{}

bool tracker::LineFollowerDetector::detectLineFollower(const cv::Mat& frame)
{
  reset();

  _lineFollowerDetector.detectMarkers(
    frame, _detectedMarkersCorners, _detectedMarkerIDs, _rejectedMarkersCorners);

  _lineFollowerDetected = hasCorrectID();

  std::cout << "Detected IDs: ";
  for (const auto& markerID: _detectedMarkerIDs)
    std::cout << markerID << " ";
  std::cout << '\n';

  std::cout << "Line follower " << _markerID << " detected: " << _lineFollowerDetected << '\n';

  return _lineFollowerDetected;
}

bool tracker::LineFollowerDetector::estimateLineFollowerPose()
{
  if (!_lineFollowerDetected || !_canEstimatePose) {
    _lineFollowerPoseEstimated = false;
    return _lineFollowerPoseEstimated;
  }

  std::cout << "Marker corners: ";
  for (const auto& corner: *_detectedMarkerCornersIterator)
    std::cout << " " << corner.x << ", " << corner.y << " ";
  std::cout << '\n';

  cv::solvePnP(
    _markerObjPoints, *_detectedMarkerCornersIterator, _camMatrix,
    _distortionCoeffs, _lineFollowerRVec, _lineFollowerTVec);

  _lineFollowerPoseEstimated = true;
  return _lineFollowerPoseEstimated;
}

void tracker::LineFollowerDetector::visualize(cv::Mat& frame)
{
  cv::aruco::drawDetectedMarkers(frame, _detectedMarkersCorners, _detectedMarkerIDs);

  if (!_lineFollowerPoseEstimated)
    return;

  cv::drawFrameAxes(
    frame, _camMatrix, _distortionCoeffs, _lineFollowerRVec, _lineFollowerTVec,
    _markerSide * 1.5f, 2);
}

bool tracker::LineFollowerDetector::hasCorrectID()
{
  auto foundID {
    std::find(_detectedMarkerIDs.begin(), _detectedMarkerIDs.end(), _markerID)};

  if (foundID == _detectedMarkerIDs.end()) {
    _lineFollowerDetected = false;
    return _lineFollowerDetected;
  }

  auto index = foundID - _detectedMarkerIDs.begin();
  std::cout << "Index: " << index << '\n';
  _detectedMarkerCornersIterator = _detectedMarkersCorners.begin() + index;

  _lineFollowerDetected = true;
  return _lineFollowerDetected;
}

void tracker::trackLineFollower(
  const options::MarkerDetection& detectionOptions,
  const options::BoardMarkers& boardMarkersOptions,
  const options::LineFollowerMarker& lineFollowerOptions,
  const std::string& outputFileName)
{
  bool hasOutputFile {outputFileName != "none"};
  std::ofstream posesOutputFile;

  if (hasOutputFile) {
    posesOutputFile = std::ofstream(outputFileName);
    if (!posesOutputFile.is_open())
      throw Error::CANNOT_OPEN_FILE;
  }
  else {
    std::cout << "No output directory given, poses will not be recorded."
              << '\n';
  }

  bool estimatePose {false};

  if (tracker::isNonZeroMatrix(detectionOptions.camMatrix)
      && tracker::isNonZeroMatrix(detectionOptions.distCoeffs)) {

    std::cout << "Marker pose estimation is active" << '\n';
    estimatePose = true;
  }

  std::cout << "Hit ESC key or Crtl + C to exit if a window opens." << '\n';

  cv::VideoCapture inputVideo;
  int waitTime;

  if(detectionOptions.inputFilePath != "none") {
    inputVideo.open(detectionOptions.inputFilePath);
    std::cout << "Using input source file instead of camera stream" << '\n';
    waitTime = 0;
  }
  else {
    inputVideo.open(detectionOptions.camID);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 30);
    waitTime = 10;
  }

  tracker::BoardDetector lineFollowerBoardDetector {
    detectionOptions, boardMarkersOptions, estimatePose};

  tracker::LineFollowerDetector lineFollowerDetector {
    detectionOptions, lineFollowerOptions, estimatePose};

  cv::Mat frame;
  while(inputVideo.grab()) {
    inputVideo.retrieve(frame);

    double tick = (double)cv::getTickCount();

    lineFollowerBoardDetector.detectBoard(frame);
    lineFollowerBoardDetector.estimateBoardPose();

    lineFollowerDetector.detectLineFollower(frame);
    lineFollowerDetector.estimateLineFollowerPose();

    lineFollowerBoardDetector.visualize(frame);
    lineFollowerDetector.visualize(frame);

    cv::imshow("out", frame);
    char key = (char)cv::waitKey(waitTime);
    if(key == 27) break;
  }
}

void tracker::calibrateCamera(const options::Calibration& options, const options::CalibrationOutput& output)
{
  bool showChessboardCorners {true};
  int calibrationFlags {0};
  cv::aruco::DetectorParameters detectorParams {cv::aruco::DetectorParameters()};

  cv::VideoCapture inputVideo;
  int waitTime;

  if(options.inputFilePath != "none") {
    inputVideo.open(options.inputFilePath);
    std::cout << "Using input source file instead of camera stream" << '\n';
    waitTime = 0;
  }
  else {
    inputVideo.open(options.camID);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 30);
    waitTime = 10;
  }

  auto arucoDictionary {cv::aruco::getPredefinedDictionary(options.markerDictionaryID)};
  cv::aruco::CharucoBoard board(
    cv::Size(options.squaresQuantityX, options.squaresQuantityY),
    options.squareSideMeters,
    options.markerSideMeters,
    arucoDictionary
  );

  // Charuco and Detector parameters are currently default
  cv::aruco::CharucoParameters charucoParams;
  cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

  std::vector<cv::Mat> allCharucoCorners;
  std::vector<cv::Mat> allCharucoIds;

  std::vector<std::vector<cv::Point2f>> allImagePoints;
  std::vector<std::vector<cv::Point3f>> allObjectPoints;

  std::vector<cv::Mat> allImages;
  cv::Size imageSize;

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
    cv::Mat currentCharucoCorners;
    cv::Mat currentCharucoIds;
    std::vector<cv::Point3f> currentObjectPoints;
    std::vector<cv::Point2f> currentImagePoints;

    // Detect ChArUco board
    detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

    // Draw results
    image.copyTo(imageCopy);
    if(!markerIds.empty()) {
      cv::aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if(currentCharucoCorners.total() > 3) {
      cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
    }

    cv::putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

    cv::imshow("out", imageCopy);

    // Wait for key pressed
    char key = (char)cv::waitKey(waitTime);

    if(key == 27)
      break;

    if(key == 'c' && currentCharucoCorners.total() > 3) {
      // Match image points
      board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

      if(currentImagePoints.empty() || currentObjectPoints.empty()) {
        std::cout << "Point matching failed, try again." << '\n';
        continue;
      }

      std::cout << "Frame captured" << '\n';

      allCharucoCorners.push_back(currentCharucoCorners);
      allCharucoIds.push_back(currentCharucoIds);
      allImagePoints.push_back(currentImagePoints);
      allObjectPoints.push_back(currentObjectPoints);
      allImages.push_back(image);

      imageSize = image.size();
    }
  }

  if(allCharucoCorners.size() < 4) {
    std::cerr << "Not enough corners for calibration" << '\n';
    return;
  }

  // Calibrate camera using ChArUco
  double repError = cv::calibrateCamera(
    allObjectPoints, allImagePoints, imageSize,
    output.cameraMatrix, output.distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(),
    cv::noArray(), cv::noArray(), calibrationFlags
  );
}
