#include "BoardMarkers.hpp"

namespace options {

void BoardMarkers::readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
{
  auto boardMarkersNode {cvFileObjectIn["board_markers"]};

  if (boardMarkersNode.empty() || !boardMarkersNode.isMap()) {
    throw std::runtime_error(
      "Board markers section is incorrectly formatted or has missing information.");
  }

  auto markerSideMetersNode {boardMarkersNode["marker_side_meters"]};
  auto markerSeperationMetersXNode {boardMarkersNode["marker_seperation_meters_x"]};
  auto markerSeperationMetersYNode {boardMarkersNode["marker_seperation_meters_y"]};
  auto markerIDsNode {boardMarkersNode["marker_ids"]};
  auto dictionaryIDNode {boardMarkersNode["marker_dictionary_id"]};

  if (!markerSideMetersNode.isReal()
    || !markerSeperationMetersXNode.isReal()
    || !markerSeperationMetersYNode.isReal()
    // || !markerIDsNode.isSeq()
    || !dictionaryIDNode.isInt()) {

    throw std::runtime_error(
      "Board markers section is incorrectly formatted or has missing information.");
  }

  cv::read(markerSideMetersNode, markerSideMeters, 0.F);
  cv::read(markerSeperationMetersXNode, markerSeperationMetersX, 0.F);
  cv::read(markerSeperationMetersYNode, markerSeperationMetersY, 0.F);
  cv::read(markerIDsNode, markerIDs, {});
  cv::read(dictionaryIDNode, markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
}

void BoardMarkers::writeToConfigFile(cv::FileStorage& cvFileObjectOut)
{
  cvFileObjectOut.writeComment("\nParameters specific to board markers.");
  cvFileObjectOut.startWriteStruct("board_markers", cv::FileNode::MAP);
    cvFileObjectOut << "marker_side_meters" << markerSideMeters;
    cvFileObjectOut << "marker_seperation_meters_x" << markerSeperationMetersX;
    cvFileObjectOut << "marker_seperation_meters_y" << markerSeperationMetersY;
    cvFileObjectOut << "marker_ids" << markerIDs;
    cvFileObjectOut << "marker_dictionary_id" << markerDictionaryID;
  cvFileObjectOut.endWriteStruct();
}

}
