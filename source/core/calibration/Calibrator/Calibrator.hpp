#include <opencv2/objdetect.hpp>

#include "Calibration.hpp"

namespace calibration {

class Calibrator {

public:
  Calibrator(const options::Calibration& optionsIn);

  void run(cv::VideoCapture& videoCaptureObjectIn, unsigned int& imageTextureOut);

  bool captureFrame();
  bool finishCalibration(options::CameraIntrinsic& paramsOut);

private:
  options::Calibration _options;

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

  std::vector<int> _markerIDs;
  std::vector<std::vector<cv::Point2f>> _markerCorners;
  std::vector<std::vector<cv::Point2f>> _rejectedMarkers;
  cv::Mat _currentCharucoCorners;
  cv::Mat _currentCharucoIDs;
  std::vector<cv::Point3f> _currentObjectPoints;
  std::vector<cv::Point2f> _currentImagePoints;

};

}
