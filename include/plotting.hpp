#pragma once

#include <opencv2/core/types.hpp>

#include <matplot/matplot.h>
// #include <matplot/backend/opengl.h>

#include "options.hpp"
#include "fileio.hpp"

namespace plotting {

class Plotter {

public:
  Plotter(const options::BoardMarkers& boardMarkersOptions);

  void setReferenceLineTrack(const options::LineTrack& referenceTrackIn);
  void setReferenceRoundTrack(const options::RoundTrack& referenceTrackIn);

  void savePosition(const cv::Point2d& positionIn);
  void saveError(double errorIn, double timeIn);
  void savePlots(const fileio::OutputPath& outputPathIn);

private:
  matplot::figure_handle _errorPlotFigure;
  matplot::figure_handle _positionPlotFigure;

  matplot::axes_handle _errorPlotAxesHandle;
  matplot::axes_handle _positionPlotAxesHandle;

  std::pair<std::vector<double>, std::vector<double>> _lineFollowerPositions;
  std::pair<std::vector<double>, std::vector<double>> _trackingErrors;
  std::pair<std::vector<double>, std::vector<double>> _referenceTrackPoints;

};

}

// auto plotFigure {matplot::figure<matplot::backend::opengl>(true)};
// auto errorAxesHandle {plotFigure->add_subplot(2, 1, 0)};
// auto positionAxesHandle {plotFigure->add_subplot(2, 1, 1)};
// errorAxesHandle->title("Error Plot");
// positionAxesHandle->title("Position");
// std::pair<std::vector<double>, std::vector<double>> lineFollowerPositions;
// std::pair<std::vector<double>, std::vector<double>> trackingErrors;
