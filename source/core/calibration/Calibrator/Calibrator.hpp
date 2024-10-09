#include <opencv2/objdetect.hpp>

#include "Calibration.hpp"

namespace calibration {

class Calibrator {

public:
  Calibrator(const options::Calibration& optionsIn);

  void run(cv::VideoCapture& videoCaptureObjectIn, cv::Mat& frameOut);

  bool captureFrame();
  bool finishCalibration(options::CameraIntrinsic& paramsOut);

private:
  bool _flipFrameVertical {false};
  bool _flipFrameHorizontal {false};

  // options::Calibration _options;

  // cv::aruco::DetectorParameters _detectorParams;
  cv::aruco::CharucoBoard _calibrationBoard;
  cv::aruco::CharucoDetector _calibrationBoardDetector;

  std::vector<cv::Mat> _allCharucoCorners;
  std::vector<cv::Mat> _allCharucoIds;
  std::vector<std::vector<cv::Point2f>> _allImagePoints;
  std::vector<std::vector<cv::Point3f>> _allObjectPoints;

  std::vector<cv::Mat> _allFrames;
  cv::Size _frameSize;
  cv::Mat _frame;
  cv::Mat _frameCopy;

  std::string _textToDisplay {"Press 'C' to add current frame. 'ESC' to finish and calibrate"};
  cv::Scalar _textToDisplayColor {255, 0, 0};

  std::vector<int> _markerIDs;
  std::vector<std::vector<cv::Point2f>> _markerCorners;
  std::vector<std::vector<cv::Point2f>> _rejectedMarkers;
  cv::Mat _currentCharucoCorners;
  cv::Mat _currentCharucoIDs;
  std::vector<cv::Point3f> _currentObjectPoints;
  std::vector<cv::Point2f> _currentImagePoints;

};

}
