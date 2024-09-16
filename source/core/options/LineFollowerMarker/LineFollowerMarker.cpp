#include "LineFollowerMarker.hpp"

namespace options {

void LineFollowerMarker::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto lineFollowerMarkerNode {cvFileObjectIn["line_follower_marker"]};

  if (lineFollowerMarkerNode.empty() || !lineFollowerMarkerNode.isMap()) {
    throw std::runtime_error(
      "Line follower marker section is incorrectly formatted or has missing information.");
  }

  auto markerSideMetersNode {lineFollowerMarkerNode["marker_side_meters"]};
  auto markerIDNode {lineFollowerMarkerNode["marker_id"]};
  auto markerDictionaryIDNode {lineFollowerMarkerNode["marker_dictionary_id"]};

  if (!markerSideMetersNode.isReal() || !markerIDNode.isInt() || !markerDictionaryIDNode.isInt()) {
    throw std::runtime_error(
      "Line follower marker section is incorrectly formatted or has missing information.");
  }

  cv::read(markerSideMetersNode, markerSideMeters, 0.0);
  cv::read(markerIDNode, markerID, 4);
  cv::read(markerDictionaryIDNode, markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
}

void LineFollowerMarker::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment("\nParameters specific to line follower marker.");
  cvFileObjectOut.startWriteStruct("line_follower_marker", cv::FileNode::MAP);
    cvFileObjectOut << "marker_side_meters" << markerSideMeters;
    cvFileObjectOut << "marker_id" << markerID;
    cvFileObjectOut << "marker_dictionary_id" << markerDictionaryID;
  cvFileObjectOut.endWriteStruct();
}

}
