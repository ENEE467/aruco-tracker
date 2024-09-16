#pragma once

#include <opencv2/core/types.hpp>
#include <matplot/matplot.h>

#include "File/File.hpp"
#include "Tracking.hpp"

namespace fileio {

class PositionsPlotFile : File {

public:
  PositionsPlotFile(
    const std::string& outputDirectoryIn,
    const std::string& nameIn,
    const options::Tracking& trackingOptionsIn);

  bool isOpen() const override {return true;}

  void addPoint(const cv::Point2d& positionIn);
  void savePlot();

private:
  matplot::figure_handle _positionPlotFigure;
  matplot::axes_handle _positionPlotAxesHandle;
  tracks::Track* _track {nullptr};

  std::pair<std::vector<double>, std::vector<double>> _lineFollowerPositions {};

};

}
