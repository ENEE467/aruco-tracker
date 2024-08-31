#include "ErrorsPlotFile.hpp"

namespace fileio {

ErrorsPlotFile::ErrorsPlotFile(const std::string& outputDirectoryIn, const std::string& nameIn)
: File {outputDirectoryIn, "errors-plot", nameIn, "jpg"},
  _errorPlotFigure {matplot::figure(true)},
  _errorPlotAxesHandle {_errorPlotFigure->current_axes()},
  _trackingErrors {{}, {}}
{
  _errorPlotFigure->size(1000, 500);

  _errorPlotAxesHandle->xlabel("Time (seconds)");
  _errorPlotAxesHandle->ylabel("Error (centimeters)");
  _errorPlotAxesHandle->ylim({0, 15});
  _errorPlotAxesHandle->axes_aspect_ratio(1 / 2); // Aspect ratio: Height / Width
}

void ErrorsPlotFile::addErrorAtTime(double errorIn, double timeIn)
{
  _trackingErrors.first.push_back(timeIn);
  _trackingErrors.second.push_back(100 * errorIn);
}

void ErrorsPlotFile::savePlot()
{
  _errorPlotAxesHandle->title_enhanced(false);
  _errorPlotAxesHandle->title("Errors " + _fileName);

  _errorPlotAxesHandle->plot(_trackingErrors.first, _trackingErrors.second, "-r");

  _errorPlotFigure->save(_filePath);
}

}
