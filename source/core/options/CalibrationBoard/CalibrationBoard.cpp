#include "CalibrationBoard.hpp"

namespace options {

void CalibrationBoard::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto cameraCalibrationNode {cvFileObjectIn["calibration_board"]};

  if (cameraCalibrationNode.empty() || !cameraCalibrationNode.isMap()) {
    throw std::runtime_error(
      "Calibration board section is incorrectly formatted or has missing information.");
  }

  auto markerSideNode {cameraCalibrationNode["marker_side_meters"]};
  auto squareSideNode {cameraCalibrationNode["square_side_meters"]};
  auto squaresQuantityXNode {cameraCalibrationNode["squares_quantity_x"]};
  auto squaresQuantityYNode {cameraCalibrationNode["squares_quantity_y"]};
  auto dictionaryIDNode {cameraCalibrationNode["marker_dictionary_id"]};

  if (!markerSideNode.isReal()
    || !dictionaryIDNode.isInt()
    || !squareSideNode.isReal()
    || !squaresQuantityXNode.isInt()
    || !squaresQuantityYNode.isInt()) {

    throw std::runtime_error(
      "Calibration board section is incorrectly formatted or has missing information.");
  }

  cv::read(markerSideNode, markerSideMeters, 0.F);
  cv::read(squareSideNode, squareSideMeters, 0.F);
  cv::read(squaresQuantityXNode, squaresQuantityX, 0);
  cv::read(squaresQuantityYNode, squaresQuantityY, 0);
  cv::read(dictionaryIDNode, markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
}

void CalibrationBoard::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment(
    "\nParameters specific to calibration board used for calibrating camera.");
  cvFileObjectOut.startWriteStruct("calibration_board", cv::FileNode::MAP);
    cvFileObjectOut << "marker_side_meters" << markerSideMeters;
    cvFileObjectOut << "square_side_meters" << squareSideMeters;
    cvFileObjectOut << "squares_quantity_x" << squaresQuantityX;
    cvFileObjectOut << "squares_quantity_y" << squaresQuantityY;
    cvFileObjectOut << "marker_dictionary_id" << markerDictionaryID;
  cvFileObjectOut.endWriteStruct();
}

}
