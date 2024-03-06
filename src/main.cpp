#include "tracker.hpp"

using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";

// TODO: Add a new option for camera calibration.
//! [aruco_detect_markers_keys]
const char* keys  =
        "{v        |       | Input from video or image file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }";
}
//! [aruco_detect_markers_keys]

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    // TODO: Finish encapsulating these parsed parameters in the new options struct
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    aruco::DetectorParameters detectorParams {cv::aruco::DetectorParameters()};

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = tracker::readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera parameters file" << endl;
            return 0;
        }
    }

    // TODO: Move this while loop into the trackLineFollower() method.


    return 0;
}
