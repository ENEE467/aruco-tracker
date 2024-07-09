#pragma once

#include <string>
#include <fstream>

#include "options.hpp"

namespace fileio {
class CSVFile {

public:
  CSVFile();
  CSVFile(
    const std::string& outputDirectoryIn,
    const std::string& prefixIn = "",
    const std::string& outputName = "");

  void setOutputPath(
    const std::string& outputDirectoryIn,
    const std::string& prefixIn = "",
    const std::string& outputName = ""
  );
  void writePosition(const cv::Point2d& positionIn, const int timeSecondsIn = 0);
  void writeError(const double errorIn, const double timeSecondsIn);

private:
  std::ofstream _outputCSVFile;

};

void readConfigFile(const std::string& filenameIn, options::MarkerDetection& optionsOut);
void readConfigFile(const std::string& filenameIn, options::LineFollowerMarker& optionsOut);
void readConfigFile(const std::string& filenameIn, options::BoardMarkers& optionsOut);
void readConfigFile(const std::string& filenameIn, options::Track& optionsOut);
void readConfigFile(const std::string& filenameIn, options::CalibrationBoard& optionsOut);
void readConfigFile(const std::string& filenameIn, options::CameraIntrinsic& parametersOut);

void readConfigFile(const std::string& filenameIn, options::Tracking& optionsOut);
void readConfigFile(const std::string& filenameIn, options::Calibration& optionsOut);

void writeConfigFile(
  const std::string& outputFileNameIn,
  const options::MarkerDetection& detectionOptionsIn,
  const options::LineFollowerMarker& lineFollowerOptionsIn,
  const options::BoardMarkers& boardOptionsIn,
  const options::Track& trackOptionsIn,
  const options::CalibrationBoard& calibrationBoardOptionsIn,
  const options::CameraIntrinsic& cameraCalibrationParamsIn);

std::stringstream createTimeStampedPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& extensionIn);

std::stringstream createPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& nameIn,
  const std::string& extensionIn);

}
