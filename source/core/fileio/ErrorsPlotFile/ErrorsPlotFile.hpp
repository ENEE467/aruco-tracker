#pragma once

#include <opencv2/core/types.hpp>
#include <matplot/matplot.h>

#include "File/File.hpp"
#include "Tracking.hpp"

namespace fileio {

class ErrorsPlotFile : File {

public:
  ErrorsPlotFile(const std::string& outputDirectoryIn, const std::string& nameIn);

  bool isOpen() const override {return true;}

  void addErrorAtTime(double errorIn, double timeIn);
  void savePlot();

private:
  matplot::figure_handle _errorPlotFigure;
  matplot::axes_handle _errorPlotAxesHandle;

  std::pair<std::vector<double>, std::vector<double>> _trackingErrors;

};

}
