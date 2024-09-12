#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <glad/glad.h>

#include "Calibrator.hpp"

// static GLuint matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter) {
// 	// Generate a number for our textureID's unique handle
// 	GLuint textureID;
// 	glGenTextures(1, &textureID);

// 	// Bind to our texture handle
// 	glBindTexture(GL_TEXTURE_2D, textureID);

// 	// Catch silly-mistake texture interpolation method for magnification
// 	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
// 		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
// 		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
// 		magFilter == GL_NEAREST_MIPMAP_NEAREST)
// 	{
// 		// cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
// 		magFilter = GL_LINEAR;
// 	}

// 	// Set texture interpolation methods for minification and magnification
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

// 	// Set texture clamping method
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

// 	// Set incoming texture format to:
// 	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
// 	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
// 	// Work out other mappings as required ( there's a list in comments in main() )
// 	GLenum inputColourFormat = GL_BGR;
// 	if (mat.channels() == 1)
// 	{
// 		inputColourFormat = GL_RED;
// 	}

// 	// Create the texture
// 	glTexImage2D(
//     GL_TEXTURE_2D,     // Type of texture
// 		0,                 // Pyramid level (for mip-mapping) - 0 is the top level
// 		GL_RGB,            // Internal colour format to convert to
// 		mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
// 		mat.rows,          // Image height i.e. 480 for Kinect in standard mode
// 		0,                 // Border width in pixels (can either be 1 or 0)
// 		inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
// 		GL_UNSIGNED_BYTE,  // Image data type
// 		mat.ptr());        // The actual image data itself

// // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
// 	if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
// 		minFilter == GL_LINEAR_MIPMAP_NEAREST ||
// 		minFilter == GL_NEAREST_MIPMAP_LINEAR ||
// 		minFilter == GL_NEAREST_MIPMAP_NEAREST)
// 	{
// 		glGenerateMipmap(GL_TEXTURE_2D);
// 	}

// 	return textureID;
// }

namespace calibration {

Calibrator::Calibrator(const options::Calibration& optionsIn)
: _options {optionsIn},

  _calibrationBoard {
    cv::Size(_options.calibrationBoard.squaresQuantityX, _options.calibrationBoard.squaresQuantityY),
    _options.calibrationBoard.squareSideMeters,
    _options.calibrationBoard.markerSideMeters,
    cv::aruco::getPredefinedDictionary(_options.calibrationBoard.markerDictionaryID)},

  _calibrationBoardDetector {
    _calibrationBoard, cv::aruco::CharucoParameters(), cv::aruco::DetectorParameters()}
{}

void Calibrator::run(cv::VideoCapture& videoCaptureObjectIn, cv::Mat& frameOut)
{
  if (!videoCaptureObjectIn.isOpened())
    return;

  _markerIDs.clear();
  _markerCorners.clear();
  _rejectedMarkers.clear();
  _currentObjectPoints.clear();
  _currentImagePoints.clear();

  videoCaptureObjectIn.read(_frame);

  // Detect ChArUco board
  _calibrationBoardDetector.detectBoard(_frame, _currentCharucoCorners, _currentCharucoIDs);

  // Draw results
  _frame.copyTo(_frameCopy);
  if(!_markerIDs.empty())
    cv::aruco::drawDetectedMarkers(_frameCopy, _markerCorners);

  if(_currentCharucoCorners.total() > 3)
    cv::aruco::drawDetectedCornersCharuco(_frameCopy, _currentCharucoCorners, _currentCharucoIDs);

  cv::putText(_frameCopy, _textToDisplay,
    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, _textToDisplayColor, 2);

  // imageTextureOut = matToTexture(_frameCopy, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
  _frameCopy.copyTo(frameOut);
}

bool Calibrator::captureFrame()
{
  if (_currentCharucoCorners.total() <= 3)
    return false;

  // Match image points
  _calibrationBoard.matchImagePoints(
    _currentCharucoCorners, _currentCharucoIDs, _currentObjectPoints, _currentImagePoints);

  if(_currentImagePoints.empty() || _currentObjectPoints.empty())
    return false;

  std::cout << "Frame captured" << '\n';

  _allCharucoCorners.push_back(_currentCharucoCorners);
  _allCharucoIds.push_back(_currentCharucoIDs);
  _allImagePoints.push_back(_currentImagePoints);
  _allObjectPoints.push_back(_currentObjectPoints);
  _allFrames.push_back(_frame);

  _frameSize = _frame.size();

  return true;
}

bool Calibrator::finishCalibration(options::CameraIntrinsic& paramsOut)
{
  if(_allCharucoCorners.size() < 4) {
    std::cerr << "Not enough corners for calibration" << '\n';
    return false;
  }

  // Calibrate camera using ChArUco
  double repError = cv::calibrateCamera(
    _allObjectPoints, _allImagePoints, _frameSize, paramsOut.cameraMatrix,
    paramsOut.distortionCoefficients, cv::noArray(), cv::noArray(), cv::noArray(),
    cv::noArray(), cv::noArray(), 0);

  return true;
}

}

// void calibration::calibrateCamera(options::Calibration& optionsIn)
// {
//   bool showChessboardCorners {true};
//   int calibrationFlags {0};
//   cv::aruco::DetectorParameters detectorParams {cv::aruco::DetectorParameters()};

//   cv::VideoCapture inputVideo;
//   int waitTime {10};

//   inputVideo.open(optionsIn.detection.camID);
//   inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, optionsIn.detection.frameWidthPixels);
//   inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, optionsIn.detection.frameHeightPixels);
//   inputVideo.set(cv::CAP_PROP_FPS, optionsIn.detection.frameRateFPS);

//   auto arucoDictionary {
//     cv::aruco::getPredefinedDictionary(optionsIn.calibrationBoard.markerDictionaryID)};

//   cv::aruco::CharucoBoard board(
//     cv::Size(optionsIn.calibrationBoard.squaresQuantityX, optionsIn.calibrationBoard.squaresQuantityY),
//     optionsIn.calibrationBoard.squareSideMeters,
//     optionsIn.calibrationBoard.markerSideMeters,
//     arucoDictionary
//   );

//   // Charuco and Detector parameters are currently default
//   cv::aruco::CharucoParameters charucoParams;
//   cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

//   std::vector<cv::Mat> allCharucoCorners;
//   std::vector<cv::Mat> allCharucoIds;

//   std::vector<std::vector<cv::Point2f>> allImagePoints;
//   std::vector<std::vector<cv::Point3f>> allObjectPoints;

//   std::vector<cv::Mat> allImages;
//   cv::Size imageSize;

//   while(inputVideo.grab()) {
//     cv::Mat image, imageCopy;
//     inputVideo.retrieve(image);

//     std::vector<int> markerIds;
//     std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
//     cv::Mat currentCharucoCorners;
//     cv::Mat currentCharucoIds;
//     std::vector<cv::Point3f> currentObjectPoints;
//     std::vector<cv::Point2f> currentImagePoints;

//     // Detect ChArUco board
//     detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

//     // Draw results
//     image.copyTo(imageCopy);
//     if(!markerIds.empty()) {
//       cv::aruco::drawDetectedMarkers(imageCopy, markerCorners);
//     }

//     if(currentCharucoCorners.total() > 3) {
//       cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
//     }

//     cv::putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
//         cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

//     cv::imshow("out", imageCopy);

//     // Wait for key pressed
//     char key = (char)cv::waitKey(waitTime);

//     if(key == 27)
//       break;

//     if(key == 'c' && currentCharucoCorners.total() > 3) {
//       // Match image points
//       board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

//       if(currentImagePoints.empty() || currentObjectPoints.empty()) {
//         std::cout << "Point matching failed, try again." << '\n';
//         continue;
//       }

//       std::cout << "Frame captured" << '\n';

//       allCharucoCorners.push_back(currentCharucoCorners);
//       allCharucoIds.push_back(currentCharucoIds);
//       allImagePoints.push_back(currentImagePoints);
//       allObjectPoints.push_back(currentObjectPoints);
//       allImages.push_back(image);

//       imageSize = image.size();
//     }
//   }

//   if(allCharucoCorners.size() < 4) {
//     std::cerr << "Not enough corners for calibration" << '\n';
//     return;
//   }

//   // Calibrate camera using ChArUco
//   double repError = cv::calibrateCamera(
//     allObjectPoints, allImagePoints, imageSize, optionsIn.calibrationParams.cameraMatrix,
//     optionsIn.calibrationParams.distortionCoefficients, cv::noArray(), cv::noArray(), cv::noArray(),
//     cv::noArray(), cv::noArray(), calibrationFlags);
// }
