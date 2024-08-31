#include "CameraIntrinsic.hpp"

namespace options {

CameraIntrinsic::CameraIntrinsic()
: cameraMatrix {cv::Mat::zeros(cv::Size(3, 3), CV_32F)},
  distortionCoefficients {cv::Mat::zeros(cv::Size(5, 1), CV_32F)},
  _isNonZero {isNonZeroMatrix(cameraMatrix) && isNonZeroMatrix(distortionCoefficients)}
{}

CameraIntrinsic::CameraIntrinsic(
  const cv::Mat& cameraMatrixIn,
  const cv::Mat& distortionCoefficientsIn)
: cameraMatrix {cameraMatrixIn},
  distortionCoefficients {distortionCoefficientsIn},
  _isNonZero {isNonZeroMatrix(cameraMatrix) && isNonZeroMatrix(distortionCoefficients)}
{}

void CameraIntrinsic::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto intrinsicParametersNode {cvFileObjectIn["intrinsic_parameters"]};

  if (intrinsicParametersNode.empty() || !intrinsicParametersNode.isMap()) {
    throw std::runtime_error(
      "Intrinsic parameters section is incorrectly formatted or has missing information.");
  }

  auto cameraMatrixNode {intrinsicParametersNode["camera_matrix"]};
  auto distortionCoefficientsNode {intrinsicParametersNode["distortion_coefficients"]};

  if (!cameraMatrixNode.isMap() || !distortionCoefficientsNode.isMap()) {
    throw std::runtime_error(
      "Intrinsic parameters section is incorrectly formatted or has missing information.");
  }

  cv::read(cameraMatrixNode, cameraMatrix, cv::Mat::zeros(cv::Size(3, 3), CV_32F));
  cv::read(
    distortionCoefficientsNode, distortionCoefficients, cv::Mat::zeros(cv::Size(5, 1), CV_32F));

  evaluateNonZero(); // Don't forget to check if either of the parameters are still zero
                    // matrices even after reading.
}

void CameraIntrinsic::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment("\nIntrinsic parameters from camera calibration.");
  cvFileObjectOut.startWriteStruct("intrinsic_parameters", cv::FileNode::MAP);
    cvFileObjectOut << "camera_matrix" << cameraMatrix;
    cvFileObjectOut << "distortion_coefficients" << distortionCoefficients;
  cvFileObjectOut.endWriteStruct();
}

}
