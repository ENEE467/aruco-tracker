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

  if (!parser.has("config")) {
    std::cerr << "A configuration file is required to run the program." << std::endl;
    parser.printMessage();
    return 1;
  }

  tracker::detectionOptions options {};
  tracker::readConfigFile(parser.get<std::string>("config"), options);

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  tracker::trackLineFollower(options);

  return 0;
}
