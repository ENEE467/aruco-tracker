#pragma once

#include <opencv2/highgui.hpp>

#include "BoardDetector/BoardDetector.hpp"
#include "LineFollowerDetector/LineFollowerDetector.hpp"
#include "Output.hpp"

namespace tracking {

class Tracker {

public:
  Tracker(const options::Tracking& optionsIn);

  void run(cv::VideoCapture& videoCaptureObjectIn, unsigned int& imageTextureOut);
  void writeOutput(Output& outputOut);

private:
  double calculateTrackingError(const cv::Point2d& positionIn);

  bool _isBoardPoseEstimated {false};
  bool _isLineFollowerPoseEstimated {false};
  bool _isOutputSaving {false};

  BoardDetector _trackBoardDetector;
  LineFollowerDetector _lineFollowerDetector;
  tracks::Track* _track {nullptr};
  cv::Mat _frame;

  std::chrono::_V2::system_clock::time_point _startTime {};
  std::chrono::_V2::system_clock::time_point _currentTime {};
  std::chrono::duration<double, std::ratio<1L, 1L>> _elapsedTime {};
  double _trackingError {};

};

}
