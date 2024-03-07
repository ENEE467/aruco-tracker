#include "tracker.hpp"

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";

// TODO: Add a new option for camera calibration.
//! [aruco_detect_markers_keys]
const char* keys  =
  "{config   |       | Configuration file path for the program }"
  "{v        |       | Input from video or image file, if ommited, input comes from camera }"
  "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
  "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
  "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }";
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
    return 1;
  }

  tracker::trackerOptions demo_options {};
  std::cout << "Config file path: " << parser.get<std::string>("config") << std::endl;
  tracker::readConfigFile(parser.get<std::string>("config"), demo_options);

  // TODO: Finish encapsulating these parsed parameters in the new options struct
  tracker::trackerOptions options {};

  options.camID = parser.get<int>("ci");
  options.markerSideMeters = parser.get<float>("l");

  if (parser.has("v")) {
    options.inputFilePath = parser.get<String>("v");
  }

  options.arucoDictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  if (parser.has("c")) {
    bool readOk {tracker::readCameraParameters(parser.get<string>("c"), options.camMatrix, options.distCoeffs)};

    if(!readOk) {
      cerr << "Invalid camera parameters file" << endl;
      return 0;
    }
  }

  if(!parser.check()) {
    parser.printErrors();
    return 0;
  }

  // TODO: Move this while loop into the trackLineFollower() method.
  tracker::trackLineFollower(options);

  return 0;
}
