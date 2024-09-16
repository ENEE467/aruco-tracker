#include "PositionsPlotFile.hpp"

namespace fileio {

PositionsPlotFile::PositionsPlotFile(
  const std::string& outputDirectoryIn,
  const std::string& nameIn,
  const options::Tracking& trackingOptionsIn)
: File {outputDirectoryIn, "positions-plot", nameIn, "jpg"},
  _positionPlotFigure {matplot::figure(true)},
  _positionPlotAxesHandle {_positionPlotFigure->current_axes()},
  _track {trackingOptionsIn.track.get()}
{
  _positionPlotFigure->size(1000, 640);

  _positionPlotAxesHandle->xlabel("X Position (centimeters)");
  _positionPlotAxesHandle->ylabel("Y Position (centimeters)");

  _positionPlotAxesHandle->xlim({0, trackingOptionsIn.boardMarkers.markerSeperationMetersX * 100});
  _positionPlotAxesHandle->ylim({0, trackingOptionsIn.boardMarkers.markerSeperationMetersY * 100});

  _positionPlotAxesHandle->axes_aspect_ratio(
    trackingOptionsIn.boardMarkers.markerSeperationMetersY
    / trackingOptionsIn.boardMarkers.markerSeperationMetersX);
}

void PositionsPlotFile::addPoint(const cv::Point2d& positionIn)
{
  _lineFollowerPositions.first.push_back(100 * positionIn.x);
  _lineFollowerPositions.second.push_back(100 * positionIn.y);
}

void PositionsPlotFile::savePlot()
{
  _positionPlotAxesHandle->title_enhanced(false);
  _positionPlotAxesHandle->title("Line Follower Positions " + _fileName);

  _track->plot(_positionPlotAxesHandle)->display_name("Reference Track");

  _positionPlotAxesHandle->hold(matplot::on);

  _positionPlotAxesHandle
    ->plot(_lineFollowerPositions.first, _lineFollowerPositions.second, "-o")
    ->display_name("Actual Path");

  _positionPlotAxesHandle->hold(matplot::off);
  matplot::legend(_positionPlotAxesHandle, {"", ""});

  _positionPlotFigure->save(_filePath);
}

}
