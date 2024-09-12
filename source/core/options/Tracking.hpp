#pragma once

#include <iostream>
#include <memory>

#include "BoardMarkers/BoardMarkers.hpp"
#include "CameraIntrinsic/CameraIntrinsic.hpp"
#include "LineFollowerMarker/LineFollowerMarker.hpp"
#include "MarkerDetection/MarkerDetection.hpp"

#include "LineTrack/LineTrack.hpp"
#include "ArcTrack/ArcTrack.hpp"
#include "RoundTrack/RoundTrack.hpp"
#include "PolygonTrack/PolygonTrack.hpp"
#include "PillTrack/PillTrack.hpp"

namespace options {

struct Tracking {

public:
  MarkerDetection detection {};
  LineFollowerMarker lineFollowerMarker {};
  BoardMarkers boardMarkers {};
  std::unique_ptr<tracks::Track> track {std::make_unique<tracks::Track>()};
  CameraIntrinsic intrinsicParams {};

  Tracking() {}
  Tracking(const cv::FileStorage& cvFileObjectIn) {readFromConfigFile(cvFileObjectIn);}

  void readFromConfigFile(const cv::FileStorage& cvFileObjectIn)
  {
    detection.readFromConfigFile(cvFileObjectIn);
    lineFollowerMarker.readFromConfigFile(cvFileObjectIn);
    boardMarkers.readFromConfigFile(cvFileObjectIn);
    intrinsicParams.readFromConfigFile(cvFileObjectIn);

    tracks::Type foundTrackType {tracks::Type::NONE};
    for (const auto& trackType: tracks::Types) {
      if (cvFileObjectIn[trackType.second].empty() || !cvFileObjectIn[trackType.second].isMap())
        continue;

      foundTrackType = trackType.first;
      break;
    }

    switch (foundTrackType) {

    case tracks::Type::LINE:
      std::cout << "Line track set" << '\n';
      track.reset(new tracks::LineTrack(cvFileObjectIn));
      break;

    case tracks::Type::ARC:
      std::cout << "Arc track set" << '\n';
      track.reset(new tracks::ArcTrack(cvFileObjectIn));
      break;

    case tracks::Type::ROUND:
      std::cout << "Round track set" << '\n';
      track.reset(new tracks::RoundTrack(cvFileObjectIn));
      break;

    case tracks::Type::POLYGON:
      std::cout << "Polygon track set" << '\n';
      track.reset(new tracks::PolygonTrack(cvFileObjectIn));
      break;

    case tracks::Type::PILL:
      std::cout << "Pill track set" << '\n';
      track.reset(new tracks::PillTrack(cvFileObjectIn));
      break;

    default:
      throw std::runtime_error("A valid track type isn't found in the config file");

    }
  }
};

}
