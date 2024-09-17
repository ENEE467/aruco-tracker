#include "MarkerDetection.hpp"

namespace options {

void MarkerDetection::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto markerDetectionNode {cvFileObjectIn["marker_detection"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap()) {
    throw std::runtime_error(
      "Marker detection section is incorrectly formatted or has missing information.");
  }

  auto camIDNode {markerDetectionNode["camera_id"]};
  auto frameWidthNode {markerDetectionNode["frame_width_pixels"]};
  auto frameHeightNode {markerDetectionNode["frame_height_pixels"]};
  auto frameRateNode {markerDetectionNode["frame_rate_fps"]};
  auto frameFlipVerticalNode {markerDetectionNode["frame_flip_vertical"]};
  auto frameFlipHorizontalNode {markerDetectionNode["frame_flip_horizontal"]};
  auto rejectedMarkersNode {markerDetectionNode["show_rejected_markers"]};

  if (!camIDNode.isInt()
    || !frameWidthNode.isInt()
    || !frameHeightNode.isInt()
    || !frameRateNode.isInt()
    || !frameFlipVerticalNode.isInt()
    || !frameFlipHorizontalNode.isInt()
    || !rejectedMarkersNode.isInt()) {

    throw std::runtime_error(
      "Marker detection section is incorrectly formatted or has missing information.");
  }

  cv::read(camIDNode, camID, 0);
  cv::read(frameWidthNode, frameWidthPixels, 640);
  cv::read(frameHeightNode, frameHeightPixels, 480);
  cv::read(frameRateNode, frameRateFPS, 30);
  cv::read(frameFlipVerticalNode, flipVertical, false);
  cv::read(frameFlipHorizontalNode, flipHorizontal, false);
  cv::read(rejectedMarkersNode, showRejectedMarkers, false);
}

void MarkerDetection::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment(
    "\nCommon detection parameters that apply to both detection and calibration modes.");
  cvFileObjectOut.startWriteStruct("marker_detection", cv::FileNode::MAP);
    cvFileObjectOut << "camera_id" << camID;
    cvFileObjectOut << "frame_width_pixels" << frameWidthPixels;
    cvFileObjectOut << "frame_height_pixels" << frameHeightPixels;
    cvFileObjectOut << "frame_rate_fps" << frameRateFPS;
    cvFileObjectOut << "frame_flip_vertical" << flipVertical;
    cvFileObjectOut << "frame_flip_horizontal" << flipHorizontal;
    cvFileObjectOut << "show_rejected_markers" << showRejectedMarkers;
  cvFileObjectOut.endWriteStruct();
}

}
