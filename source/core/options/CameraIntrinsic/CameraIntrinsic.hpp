#pragma once

#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/core/persistence.hpp>

namespace options {

struct CameraIntrinsic {

public:
  cv::Mat cameraMatrix;
  cv::Mat distortionCoefficients;

  CameraIntrinsic();
  CameraIntrinsic(const cv::Mat& cameraMatrixIn, const cv::Mat& distortionCoefficientsIn);

  const bool isNonZero() const {return _isNonZero;}

  void evaluateNonZero()
  {
    _isNonZero = (isNonZeroMatrix(cameraMatrix) && isNonZeroMatrix(distortionCoefficients));
  }

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn);
  void writeToConfigFile(cv::FileStorage& cvFileObjectOut);

private:
  bool _isNonZero;

  bool isNonZeroMatrix(const cv::Mat& matrix)
  {
    cv::Mat zeroMatrix {cv::Mat::zeros(matrix.rows, matrix.cols, matrix.type())};
    auto comparisonMatrix {matrix != zeroMatrix};

    return cv::countNonZero(comparisonMatrix);
  }

};

}
