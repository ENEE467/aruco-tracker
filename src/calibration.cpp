#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

#include "calibration.hpp"

void calibration::calibrateCamera(options::Calibration& optionsIn)
{
  bool showChessboardCorners {true};
  int calibrationFlags {0};
  cv::aruco::DetectorParameters detectorParams {cv::aruco::DetectorParameters()};

  cv::VideoCapture inputVideo;
  int waitTime {10};

  inputVideo.open(optionsIn.detection.camID);
  inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, optionsIn.detection.frameWidthPixels);
  inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, optionsIn.detection.frameHeightPixels);
  inputVideo.set(cv::CAP_PROP_FPS, optionsIn.detection.frameRateFPS);

  auto arucoDictionary {
    cv::aruco::getPredefinedDictionary(optionsIn.calibrationBoard.markerDictionaryID)};

  cv::aruco::CharucoBoard board(
    cv::Size(optionsIn.calibrationBoard.squaresQuantityX, optionsIn.calibrationBoard.squaresQuantityY),
    optionsIn.calibrationBoard.squareSideMeters,
    optionsIn.calibrationBoard.markerSideMeters,
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
    allObjectPoints, allImagePoints, imageSize, optionsIn.calibrationParams.cameraMatrix,
    optionsIn.calibrationParams.distortionCoefficients, cv::noArray(), cv::noArray(), cv::noArray(),
    cv::noArray(), cv::noArray(), calibrationFlags);
}
