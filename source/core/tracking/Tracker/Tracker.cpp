#include <glad/glad.h>

#include <opencv2/imgproc.hpp>

#include "Tracker.hpp"

namespace tracking {

Tracker::Tracker(
  const options::Tracking& optionsIn)
: _flipFrameVertical {optionsIn.detection.flipVertical},
  _flipFrameHorizontal {optionsIn.detection.flipHorizontal},
  _trackBoardDetector {optionsIn},
  _lineFollowerDetector {optionsIn},
  _track {optionsIn.track.get()}
{}

void tracking::Tracker::run(
  cv::VideoCapture& videoCaptureObjectIn,
  cv::Mat& frameOut)
{
  if (!videoCaptureObjectIn.isOpened())
   return;

  videoCaptureObjectIn.read(_frame);

  if (_flipFrameVertical && !_flipFrameHorizontal)
    cv::flip(_frame, _frame, 0);

  else if (!_flipFrameVertical && _flipFrameHorizontal)
    cv::flip(_frame, _frame, 1);

  else if (_flipFrameVertical && _flipFrameHorizontal)
    cv::flip(_frame, _frame, -1);

  _trackBoardDetector.detectBoard(_frame);
  _isBoardPoseEstimated = _trackBoardDetector.estimateFrameBoard_Camera();

  _lineFollowerDetector.detectLineFollower(_frame);

  bool isLineFollower_CameraPoseEstimated {_lineFollowerDetector.estimateFrameLineFollower_Camera()};

  if (isLineFollower_CameraPoseEstimated) {
    _isLineFollowerPoseEstimated = _trackBoardDetector.estimateFrameLineFollower_Board(
      _lineFollowerDetector.getFrameLineFollower_Camera());
  }

  if (_isLineFollowerPoseEstimated) {
    _trackingError =
      _track->calculatePerpendicularDistance(_trackBoardDetector.getPositionXYLineFollower_Board());
  }
  else {
    _trackingError = 0;
  }

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

  // imageTextureOut = matToTexture(_frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
  _frame.copyTo(frameOut);

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
