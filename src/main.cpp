#include <iostream>

#include "fileio.hpp"
#include "tracker.hpp"

using namespace std;
using namespace cv;

namespace {
const char* about = "Line follower tracker using ArUco markers";

//! [aruco_detect_markers_keys]
const char* keys  =
  "{config               |       | Configuration file path for the program }"
  "{calibration          | false | Enable calibration mode }"
  "{output               |       | Output file directory for a new configuration file }";
}
//! [aruco_detect_markers_keys]

int main(int argc, char *argv[]) {
  CommandLineParser parser(argc, argv, keys);
  parser.about(about);

  // if (argc < 2) {
  //   parser.printMessage();
  //   return 0;
  // }

  bool hasConfigFile {parser.has("config")};
  bool hasOutputDir {parser.has("output")};
  bool calibrationMode {parser.get<bool>("calibration")};

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  if (!hasConfigFile && !calibrationMode) {
    std::cerr << "A configuration file is required for tracking." << std::endl;
    parser.printMessage();
    return 1;
  }
  else if (!hasConfigFile && !hasOutputDir && calibrationMode) {
    std::cerr << "At least an output directory is required for calibration.";
    parser.printMessage();
    return 1;
  }

  // TODO: Finish this.
  options::MarkerDetection detectionOptions {};
  options::LineFollowerMarker lineFollowerMarkerOptions {};
  options::BoardMarkers boardMarkersOptions{};
  options::Calibration calibrationOptions {};
  options::CalibrationOutput calibrationOutput {};

  if (!calibrationMode) {
    auto fileName {parser.get<std::string>("config")};
    fileio::readConfigFile(fileName, detectionOptions);
    fileio::readConfigFile(fileName, lineFollowerMarkerOptions);
    fileio::readConfigFile(fileName, boardMarkersOptions);

    if (hasOutputDir) {
      auto outputDir {parser.get<std::string>("output")};
      auto fileName {fileio::createTimeStampedFileName(outputDir, "run", "csv")};
      tracker::trackLineFollower(
        detectionOptions, boardMarkersOptions, lineFollowerMarkerOptions, fileName.str());
    }
    else {
      tracker::trackLineFollower(
        detectionOptions, boardMarkersOptions, lineFollowerMarkerOptions);
    }
  }
  else {
    if (hasConfigFile) {
      auto fileName {parser.get<std::string>("config")};
      fileio::readConfigFile(fileName, detectionOptions);
      fileio::readConfigFile(fileName, boardMarkersOptions);
      fileio::readConfigFile(fileName, calibrationOptions);
      tracker::calibrateCamera(calibrationOptions, calibrationOutput);

      if (hasOutputDir) {
        auto outputDir {parser.get<std::string>("output")};
        auto fileNameStream {fileio::createTimeStampedFileName(outputDir, "config", "yaml")};
        fileio::writeConfigFile(fileNameStream.str(), detectionOptions, lineFollowerMarkerOptions, boardMarkersOptions, calibrationOptions, calibrationOutput);
      }
      else {
        std::remove(fileName.c_str());
        fileio::writeConfigFile(fileName, detectionOptions, lineFollowerMarkerOptions, boardMarkersOptions, calibrationOptions, calibrationOutput);
      }

      return 0;
    }
    else if (!hasConfigFile && hasOutputDir) {
      std::cout << "Creating a new config file in the output directory..." << std::endl;
      auto outputDir {parser.get<std::string>("output")};

      auto fileNameStream {fileio::createTimeStampedFileName(outputDir, "config", "yaml")};

      fileio::writeConfigFile(fileNameStream.str(), detectionOptions, lineFollowerMarkerOptions, boardMarkersOptions, calibrationOptions, calibrationOutput);
      std::cout << "Config file created: " << fileNameStream.str() << std::endl;

      return 0;
    }
  }

  return 0;
}
