#include "plotting.hpp"
#include "errors.hpp"
#include "fileio.hpp"

plotting::Plotter::Plotter(const options::BoardMarkers& boardMarkersOptions)
: _errorPlotFigure {matplot::figure(true)},
  _positionPlotFigure {matplot::figure(true)},
  _errorPlotAxesHandle {_errorPlotFigure->current_axes()},
  _positionPlotAxesHandle {_positionPlotFigure->current_axes()},
  _lineFollowerPositions {{}, {}},
  _trackingErrors {{}, {}},
  _referenceTrackPoints {{}, {}}
{
  // TODO: Do some initial testing and find the maximum time needed to track poses in each run.
  //       Use that to reserve those many points for _lineFollowerPositions and _trackErrors vectors.
  _errorPlotFigure->size(1000, 500);
  _positionPlotFigure->size(1000, 640);

  _errorPlotAxesHandle->xlabel("Time (seconds)");
  _errorPlotAxesHandle->ylabel("Error (centimeters)");
  _errorPlotAxesHandle->ylim({0, 15});
  _errorPlotAxesHandle->axes_aspect_ratio(1 / 2); // Aspect ratio: Height / Width

  _positionPlotAxesHandle->xlabel("X Position (centimeters)");
  _positionPlotAxesHandle->ylabel("Y Position (centimeters)");

  _positionPlotAxesHandle->xlim({0, boardMarkersOptions.markerSeperationMetersX * 100});
  _positionPlotAxesHandle->ylim({0, boardMarkersOptions.markerSeperationMetersY * 100});
  _positionPlotAxesHandle->axes_aspect_ratio(
    boardMarkersOptions.markerSeperationMetersY / boardMarkersOptions.markerSeperationMetersX);
}

void plotting::Plotter::setReferenceTrack(const options::Track& trackOptionsIn)
{
  switch (trackOptionsIn.selection) {

  case options::TrackSelection::LINE:
    setReferenceLineTrack(trackOptionsIn.lineTrack);
    break;

  case options::TrackSelection::ROUND:
    setReferenceRoundTrack(trackOptionsIn.roundTrack);
    break;

  default:
    throw Error::INVALID_TRACK_OPTION;

  }
}

void plotting::Plotter::setReferenceLineTrack(const options::LineTrack& referenceTrackIn)
{
  _referenceTrackPoints.first.clear();
  _referenceTrackPoints.second.clear();
  _referenceTrackPoints.first.reserve(2);
  _referenceTrackPoints.second.reserve(2);

  _referenceTrackPoints.first = {referenceTrackIn.point1.x * 100, referenceTrackIn.point2.x * 100};
  _referenceTrackPoints.second = {referenceTrackIn.point1.y * 100, referenceTrackIn.point2.y * 100};
}

void plotting::Plotter::setReferenceRoundTrack(const options::RoundTrack& referenceTrackIn)
{
  _referenceTrackPoints.first.clear();
  _referenceTrackPoints.second.clear();
  _referenceTrackPoints.first.reserve(361);
  _referenceTrackPoints.second.reserve(361);

  for (double theta {0.0}; theta < 2 * M_PI; theta += M_PI / 180) {
    _referenceTrackPoints.first.push_back(100 * referenceTrackIn.semiMajorAxis() * std::cos(theta));
    _referenceTrackPoints.second.push_back(100 * referenceTrackIn.semiMinorAxis() * std::sin(theta));
  }
}

void plotting::Plotter::savePosition(const cv::Point2d& positionIn)
{
  _lineFollowerPositions.first.push_back(100 * positionIn.x);
  _lineFollowerPositions.second.push_back(100 * positionIn.y);
}

void plotting::Plotter::saveError(double errorIn, double timeIn)
{
  _trackingErrors.first.push_back(timeIn);
  _trackingErrors.second.push_back(100 * errorIn);
}

void plotting::Plotter::savePlots(
  const std::string& outputDirectoryIn,
  const std::string& outputNameIn)
{
  std::stringstream errorPlotPath {
    fileio::createPath(outputDirectoryIn, "error-plot", outputNameIn, "jpg")};

  _errorPlotAxesHandle->title_enhanced(false);
  _errorPlotAxesHandle->title("Error Plot " + outputNameIn);

  _errorPlotAxesHandle->plot(_trackingErrors.first, _trackingErrors.second, "-r");

  _errorPlotFigure->save(errorPlotPath.str());

  std::stringstream positionPlotPath {
    fileio::createPath(outputDirectoryIn, "positions-plot", outputNameIn, "jpg")};

  _positionPlotAxesHandle->title_enhanced(false);
  _positionPlotAxesHandle->title("Line Follower Position " + outputPathIn.outputName);

  _positionPlotAxesHandle->plot(_referenceTrackPoints.first, _referenceTrackPoints.second, "g");
  _positionPlotAxesHandle->hold(matplot::on);
  _positionPlotAxesHandle->plot(_lineFollowerPositions.first, _lineFollowerPositions.second, "-o");
  _positionPlotAxesHandle->hold(matplot::off);
  _positionPlotAxesHandle->legend({"Reference Path", "Actual Path"});

  _positionPlotFigure->save(positionPlotPath.str());
}
