#include <iostream>

#include "core/tracking.hpp"
#include "core/calibration.hpp"

namespace {

const char* about = "Line follower tracking using ArUco markers";

//! [aruco_detect_markers_keys]
const char* keys  =
  "{config               |       | Configuration file path for the program }"
  "{calibration          | false | Enable calibration mode }"
  "{output               |       | Output file directory for a new configuration file }"
  "{name                 |       | Name of the output}";

}
//! [aruco_detect_markers_keys]

int main(int argc, char *argv[]) {
  cv::CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  // if (argc < 2) {
  //   parser.printMessage();
  //   return 0;
  // }

  bool hasConfigFile {parser.has("config")};
  bool hasOutputDir {parser.has("output")};
  bool hasOutputName {parser.has("name")};
  bool calibrationMode {parser.get<bool>("calibration")};

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  if (!hasConfigFile && !calibrationMode) {
    std::cerr << "A configuration file is required for tracking." << '\n';
    parser.printMessage();
    return 1;
  }
  else if (!hasConfigFile && !hasOutputDir && calibrationMode) {
    std::cerr << "At least an output directory is required for calibration." << '\n';
    parser.printMessage();
    return 1;
  }

  std::string configFilePath {};
  if (hasConfigFile)
    configFilePath = parser.get<std::string>("config");

  std::string outputDir {};
  if (hasOutputDir)
    outputDir = parser.get<std::string>("output");

  std::string outputName {};
  if (hasOutputName)
    outputName = parser.get<std::string>("name");

  options::Tracking trackingOptions {};
  options::Calibration calibrationOptions {};

  if (!calibrationMode) {
    fileio::readConfigFile(configFilePath, trackingOptions);
    tracking::trackLineFollower(trackingOptions, outputDir, outputName);
  }
  else {
    std::string outputConfigPath {};
    if (hasConfigFile) {
      outputConfigPath = configFilePath;
      fileio::readConfigFile(configFilePath, trackingOptions);
      fileio::readConfigFile(configFilePath, calibrationOptions);

      calibration::calibrateCamera(calibrationOptions);
    }

    // std::string outputConfigPath {configFilePath};
    if (hasOutputDir) {
      outputConfigPath = fileio::createPath(outputDir, "config", outputName, "yaml").str();

      std::cout << "Creating a new config file at: " << outputConfigPath << '\n';
    }
    else if (hasConfigFile && !hasOutputDir) {
      std::remove(outputConfigPath.c_str());
    }

    fileio::writeConfigFile(
      outputConfigPath,
      trackingOptions.detection,
      trackingOptions.lineFollowerMarker,
      trackingOptions.boardMarkers,
      trackingOptions.track,
      calibrationOptions.calibrationBoard,
      calibrationOptions.calibrationParams);
  }

  return 0;
}
