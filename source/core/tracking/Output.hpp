#pragma once

#include <memory>
#include <filesystem>

#include "CSVFile/CSVFile.hpp"
#include "ErrorsPlotFile/ErrorsPlotFile.hpp"
#include "PositionsPlotFile/PositionsPlotFile.hpp"

namespace tracking {

struct Output {

public:
  std::unique_ptr<fileio::CSVFile> positionsCSV {};
  std::unique_ptr<fileio::CSVFile> errorsCSV {};
  std::unique_ptr<fileio::PositionsPlotFile> positionsPlot {};
  std::unique_ptr<fileio::ErrorsPlotFile> errorsPlot {};

  Output(const std::string& outputParentDirectoryPathIn, const std::string& outputNameIn = "")
  : _outputParentDirectoryPath {outputParentDirectoryPathIn},
    _outputName {outputNameIn}
  {}

  void open(const options::Tracking& trackingOptionsIn)
  {
    _outputDirectoryPath = fileio::createPath(_outputParentDirectoryPath, "run", _outputName, "");
    std::filesystem::create_directory(_outputDirectoryPath);

    positionsCSV.reset(new fileio::CSVFile(_outputDirectoryPath, "positions", _outputName));
    errorsCSV.reset(new fileio::CSVFile(_outputDirectoryPath, "errors", _outputName));

    positionsPlot.reset(
      new fileio::PositionsPlotFile(_outputDirectoryPath, _outputName, trackingOptionsIn));

    errorsPlot.reset(
      new fileio::ErrorsPlotFile(_outputDirectoryPath, _outputName));

    _isOpen =
      positionsCSV->isOpen() && errorsCSV->isOpen()
      && positionsPlot->isOpen() && errorsPlot->isOpen();
  }

  const bool isOpen() const {return _isOpen;}

  void close()
  {
    positionsCSV.reset(nullptr);
    errorsCSV.reset(nullptr);

    positionsPlot->savePlot();
    positionsPlot.reset(nullptr);

    errorsPlot->savePlot();
    errorsPlot.reset(nullptr);

    _outputDirectoryPath.clear();
    _outputName.clear();
    _isOpen = false;
  }

private:
  bool _isOpen {false};
  std::string _outputParentDirectoryPath {};
  std::string _outputDirectoryPath {};
  std::string _outputName {};

};

}
