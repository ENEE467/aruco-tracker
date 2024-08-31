#include <glad/glad.h>

#include <opencv2/imgproc.hpp>

#include "Tracker.hpp"

namespace tracking {

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

Tracker::Tracker(
  const options::Tracking& optionsIn)
: _track {optionsIn.track.get()},
  _trackBoardDetector {optionsIn},
  _lineFollowerDetector {optionsIn}
{}

void tracking::Tracker::run(
  cv::VideoCapture& videoCaptureObjectIn,
  unsigned int& imageTextureOut)
{
  if (!videoCaptureObjectIn.isOpened())
   return;

  videoCaptureObjectIn.read(_frame);

  _trackBoardDetector.detectBoard(_frame);
  _isBoardPoseEstimated = _trackBoardDetector.estimateFrameBoard_Camera();

  _lineFollowerDetector.detectLineFollower(_frame);

  bool isLineFollower_CameraPoseEstimated {_lineFollowerDetector.estimateFrameLineFollower_Camera()};

  if (isLineFollower_CameraPoseEstimated) {
    _isLineFollowerPoseEstimated = _trackBoardDetector.estimateFrameLineFollower_Board(
      _lineFollowerDetector.getFrameLineFollower_Camera());
  }

  _trackingError =
    _track->calculatePerpendicularDistance(_trackBoardDetector.getPositionXYLineFollower_Board());

  _trackBoardDetector.visualize(_frame, _track);
  _lineFollowerDetector.visualize(_frame);

  auto trackingStatusTextSize {
    cv::getTextSize(
      _isOutputSaving ? "Tracking, press SPACE to stop" : "Not Tracking, press SPACE to start",
      cv::FONT_HERSHEY_DUPLEX, 0.5, 2, 0)};

  auto trackingStatusTextPosition {
    cv::Point(
      (_frame.cols - trackingStatusTextSize.width) * 0.5,
      (_frame.rows - trackingStatusTextSize.height) * 0.975)};

  cv::putText(
    _frame,
    _isOutputSaving ? "Tracking, press SPACE to stop" : "Not Tracking, press SPACE to start",
    trackingStatusTextPosition,
    cv::FONT_HERSHEY_DUPLEX, 0.5,
    _isOutputSaving ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0));

  imageTextureOut = matToTexture(_frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);

  // char key {static_cast<char>(cv::waitKey(10))};
  // if(key == 27) {
  //   break;
  // }
  // else if (key == 32) {
  // if (key == 32) {
  //   _saveOutput = !_saveOutput;

  //   if (_saveOutput) {
  //     _trackingOutput.open(_options, _outputParentDirectory, _outputName);
  //     _startTime = std::chrono::high_resolution_clock::now();
  //     std::cout << "Tracking has begun, good luck!" << '\n';
  //   }
  //   else if ((!_saveOutput)) {
  //     _trackingOutput.close();
  //     std::cout << "Tracking has ended, hit SPACE again to track a new run." << '\n';
  //   }
  // }

  // Output specific stuff only from here.
  // if (!_trackingOutput.isOpen())
  //   return;

  // _currentTime = std::chrono::high_resolution_clock::now();
  // _elapsedTime = _currentTime - _startTime;

  // _trackingOutput.errorsCSV->writeError(trackingError, _elapsedTime.count());

  // _trackingOutput.positionsCSV->writePosition(
  //   _trackBoardDetector.getPositionXYLineFollower_Board(), _elapsedTime.count());

  // _trackingOutput.errorsPlot->addErrorAtTime(trackingError, _elapsedTime.count());
  // _trackingOutput.positionsPlot->addPoint(_trackBoardDetector.getPositionXYLineFollower_Board());
}

void Tracker::writeOutput(Output& outputOut)
{
  if (!outputOut.isOpen()) {
    _isOutputSaving = false;
    return;
  }

  if (!_isOutputSaving) {
    _startTime = std::chrono::high_resolution_clock::now();
    _isOutputSaving = true;
  }

  _currentTime = std::chrono::high_resolution_clock::now();
  _elapsedTime = _currentTime - _startTime;

  outputOut.errorsCSV->writeError(_trackingError, _elapsedTime.count());

  outputOut.positionsCSV->writePosition(
    _trackBoardDetector.getPositionXYLineFollower_Board(), _elapsedTime.count());

  outputOut.errorsPlot->addErrorAtTime(_trackingError, _elapsedTime.count());
  outputOut.positionsPlot->addPoint(_trackBoardDetector.getPositionXYLineFollower_Board());
}

}
