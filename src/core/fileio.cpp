#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <opencv2/core/persistence.hpp>

#include "core/errors.hpp"
#include "core/fileio.hpp"

fileio::CSVFile::CSVFile()
: _outputCSVFile {}
{}

fileio::CSVFile::CSVFile(
  const std::string& outputDirectoryIn,
  const std::string& prefixIn,
  const std::string& outputNameIn
)
: _outputCSVFile {
    createPath(outputDirectoryIn, prefixIn, outputNameIn, "csv").str()}
{
  if (!_outputCSVFile.is_open())
    throw Error::CANNOT_OPEN_FILE;
}

void fileio::CSVFile::setOutputPath(
  const std::string& outputDirectoryIn,
  const std::string& prefixIn,
  const std::string& outputNameIn)
{
  if (outputDirectoryIn.empty())
    return;

  _outputCSVFile.open(
    createPath(outputDirectoryIn, prefixIn, outputNameIn, "csv").str());

  if (!_outputCSVFile.is_open())
    throw Error::CANNOT_OPEN_FILE;
}

void fileio::CSVFile::writePosition(const cv::Point2d& positionIn, const int timeSecondsIn)
{
  _outputCSVFile << positionIn.x << ", " << positionIn.y << ", " << timeSecondsIn << '\n';
}

void fileio::CSVFile::writeError(const double errorIn, const double timeSecondsIn)
{
  _outputCSVFile << errorIn << ", " << timeSecondsIn << '\n';
}

void fileio::readConfigFile(const std::string& filenameIn, options::MarkerDetection& optionsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto markerDetectionNode {configFile["marker_detection"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap())
    throw Error::MISSING_MARKER_DETECTION_CONFIG;

  // TODO: Make it more robust by adding read checks for individual parameters

  auto camIDNode {markerDetectionNode["camera_id"]};
  auto frameWidth {markerDetectionNode["frame_width_pixels"]};
  auto frameHeight {markerDetectionNode["frame_height_pixels"]};
  auto frameRate {markerDetectionNode["frame_rate_fps"]};
  auto rejectedMarkersNode {configFile["rejected_markers"]};

  cv::read(camIDNode, optionsOut.camID, 0);
  cv::read(frameWidth, optionsOut.frameWidthPixels, 640);
  cv::read(frameHeight, optionsOut.frameHeightPixels, 480);
  cv::read(frameRate, optionsOut.frameRateFPS, 30);
  cv::read(rejectedMarkersNode, optionsOut.showRejectedMarkers, false);
}

void fileio::readConfigFile(const std::string& filenameIn, options::LineFollowerMarker& optionsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto lineFollowerMarkerNode {configFile["line_follower_marker"]};

  if (lineFollowerMarkerNode.empty() || !lineFollowerMarkerNode.isMap())
    throw Error::MISSING_LINE_FOLLOWER_MARKER_CONFIG;

  // TODO: Make it more robust by adding read checks for individual parameters

  auto markerSideMetersNode {lineFollowerMarkerNode["marker_side_meters"]};
  auto markerIDNode {lineFollowerMarkerNode["marker_id"]};
  auto markerDictionaryID {lineFollowerMarkerNode["marker_dictionary_id"]};

  cv::read(markerSideMetersNode, optionsOut.markerSideMeters, 0.0);
  cv::read(markerIDNode, optionsOut.markerID, 4);
  cv::read(markerDictionaryID, optionsOut.markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
}

void fileio::readConfigFile(const std::string& filenameIn, options::BoardMarkers& optionsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  // auto markerDetectionNode {configFile["marker_detection"]};
  auto boardParametersNode {configFile["board_markers"]};

  if (boardParametersNode.empty() || !boardParametersNode.isMap())
    throw Error::MISSING_BOARD_MARKERS_CONFIG;

  // TODO: Make it more robust by adding read checks for individual parameters

  auto markerSideMetersNode {boardParametersNode["marker_side_meters"]};
  auto markerSeperationMetersXNode {boardParametersNode["marker_seperation_meters_x"]};
  auto markerSeperationMetersYNode {boardParametersNode["marker_seperation_meters_y"]};
  auto markerIDsNode {boardParametersNode["marker_ids"]};
  auto dictionaryIDNode {boardParametersNode["marker_dictionary_id"]};

  cv::read(markerSideMetersNode, optionsOut.markerSideMeters, 0.F);
  cv::read(markerSeperationMetersXNode, optionsOut.markerSeperationMetersX, 0.F);
  cv::read(markerSeperationMetersYNode, optionsOut.markerSeperationMetersY, 0.F);
  cv::read(markerIDsNode, optionsOut.markerIDs, {});
  cv::read(dictionaryIDNode, optionsOut.markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
}

void fileio::readConfigFile(const std::string& filenameIn, options::Track& optionsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  options::TrackSelection trackSelection {static_cast<int>(configFile["track_selection"])};
  auto lineTrackNode {configFile["line_track"]};
  auto roundTrackNode {configFile["round_track"]};

  // TODO: Make it more robust by adding read checks for individual parameters

  // Configuration error checking
  switch (trackSelection) {

  case options::TrackSelection::LINE:
    if (lineTrackNode.empty() || !lineTrackNode.isMap())
      throw Error::MISSING_LINE_TRACK_CONFIG;
    break;

  case options::TrackSelection::ROUND:
    if (roundTrackNode.empty() || !roundTrackNode.isMap())
      throw Error::MISSING_ROUND_TRACK_CONFIG;
    break;

  default:
    throw::Error::INVALID_TRACK_OPTION;

  }

  optionsOut.selection = trackSelection;

  cv::Point2d readLineTrackPoint1 {0, 0};
  cv::Point2d readLineTrackPoint2 {0, 0};
  cv::Point2d readRoundTrackCenter {0, 0};
  double readRoundTrackMajorAxis {0};
  double readRoundTrackMinorAxis {0};

  // Read config file
  switch (optionsOut.selection) {

  case options::TrackSelection::LINE:
    cv::read(lineTrackNode["point1"]["x_meters"], readLineTrackPoint1.x, 0.0);
    cv::read(lineTrackNode["point1"]["y_meters"], readLineTrackPoint1.y, 0.0);
    cv::read(lineTrackNode["point2"]["x_meters"], readLineTrackPoint2.x, 0.0);
    cv::read(lineTrackNode["point2"]["y_meters"], readLineTrackPoint2.y, 0.0);

    optionsOut.lineTrack.setPoints(readLineTrackPoint1, readLineTrackPoint2);

    break;

  case options::TrackSelection::ROUND:
    cv::read(roundTrackNode["center"]["x_meters"], readRoundTrackCenter.x, 0.0);
    cv::read(roundTrackNode["center"]["y_meters"], readRoundTrackCenter.y, 0.0);
    cv::read(roundTrackNode["width_meters"], readRoundTrackMajorAxis, 0.0);
    cv::read(roundTrackNode["height_meters"], readRoundTrackMinorAxis, 0.0);

    optionsOut.roundTrack.setParameters(
      readRoundTrackCenter, readRoundTrackMajorAxis, readRoundTrackMinorAxis);

    break;

  default:
    throw Error::INVALID_TRACK_OPTION;

  }
}

void fileio::readConfigFile(const std::string& filenameIn, options::CalibrationBoard& optionsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto cameraCalibrationNode {configFile["camera_calibration"]};

  if (cameraCalibrationNode.empty() || !cameraCalibrationNode.isMap())
    throw Error::MISSING_CALIBRATION_CONFIG;

  // TODO: Make it more robust by adding read checks for individual parameters

  auto markerSideNode {cameraCalibrationNode["marker_side_meters"]};
  auto dictionaryIDNode {cameraCalibrationNode["marker_dictionary_id"]};
  auto squareSideNode {cameraCalibrationNode["square_side_meters"]};
  auto squaresQuantityXNode {cameraCalibrationNode["squares_quantity_x"]};
  auto squaresQuantityYNode {cameraCalibrationNode["squares_quantity_y"]};

  cv::read(markerSideNode, optionsOut.markerSideMeters, 0.F);
  cv::read(dictionaryIDNode, optionsOut.markerDictionaryID, cv::aruco::DICT_ARUCO_MIP_36h12);
  cv::read(squareSideNode, optionsOut.squareSideMeters, 0.F);
  cv::read(squaresQuantityXNode, optionsOut.squaresQuantityX, 0);
  cv::read(squaresQuantityYNode, optionsOut.squaresQuantityY, 0);
}

void fileio::readConfigFile(const std::string& filenameIn, options::CameraIntrinsic& paramsOut)
{
  cv::FileStorage configFile(filenameIn, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto cameraCalibrationNode {configFile["camera_calibration"]};

  if (cameraCalibrationNode.empty() || !cameraCalibrationNode.isMap())
    throw Error::MISSING_CALIBRATION_CONFIG;

  // TODO: Make it more robust by adding read checks for individual parameters

  auto cameraMatrixNode {cameraCalibrationNode["camera_matrix"]};
  auto distortionCoefficientsNode {cameraCalibrationNode["distortion_coefficients"]};

  cv::read(cameraMatrixNode, paramsOut.cameraMatrix, cv::Mat::zeros(cv::Size(3, 3), CV_32F));
  cv::read(
    distortionCoefficientsNode, paramsOut.distortionCoefficients,
    cv::Mat::zeros(cv::Size(5, 1), CV_32F));

  paramsOut.evaluateNonZero(); // Don't forget to check if either of the parameters are still zero
                               // matrices even after reading.
}

void fileio::readConfigFile(const std::string& filenameIn, options::Tracking& optionsOut)
{
  readConfigFile(filenameIn, optionsOut.detection);
  readConfigFile(filenameIn, optionsOut.lineFollowerMarker);
  readConfigFile(filenameIn, optionsOut.boardMarkers);
  readConfigFile(filenameIn, optionsOut.track);
  readConfigFile(filenameIn, optionsOut.calibrationParams);
}

void fileio::readConfigFile(const std::string& filenameIn, options::Calibration& optionsOut)
{
  readConfigFile(filenameIn, optionsOut.detection);
  readConfigFile(filenameIn, optionsOut.calibrationBoard);
  readConfigFile(filenameIn, optionsOut.calibrationParams);
}

std::stringstream fileio::createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  return timeStamp;
}

std::stringstream fileio::createTimeStampedPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& extensionIn)
{
  std::string parentDirectory {parentDirectoryIn};
  if (parentDirectory.back() != '/')
    parentDirectory.push_back('/');

  std::string prefix {prefixIn};
  if (!prefix.empty())
    prefix.push_back('-');

  std::string fileExtension {};
  if (!extensionIn.empty())
    fileExtension = extensionIn.front() == '.' ? "" : '.' + extensionIn;

  std::stringstream outputPathStream;
  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};

  outputPathStream << parentDirectory << prefix << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
             << fileExtension;

  return outputPathStream;
}

std::stringstream fileio::createPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& nameIn,
  const std::string& extensionIn)
{
  if (nameIn.empty())
    return createTimeStampedPath(parentDirectoryIn, prefixIn, extensionIn);

  std::string parentDirectory {parentDirectoryIn};
  if (parentDirectory.back() != '/')
    parentDirectory.push_back('/');

  std::string prefix {prefixIn};
  if (!prefix.empty() && prefix.back() != '-')
    prefix.push_back('-');

  bool isPathToDirectory {true};
  std::string fileExtension {};
  if (!extensionIn.empty()) {
    isPathToDirectory = false;
    fileExtension = extensionIn.front() == '.' ? "" : '.' + extensionIn;
  }

  auto pathAlreadyExists = [&isPathToDirectory](const std::stringstream& outputPathIn)
  {
    return ( isPathToDirectory && std::filesystem::is_directory(outputPathIn.str()) )
      || ( !isPathToDirectory && std::filesystem::is_regular_file(outputPathIn.str()) );
  };

  int duplicateCounter {1};
  std::stringstream outputPathStream;
  outputPathStream << parentDirectory << prefix << nameIn << fileExtension;

  while (pathAlreadyExists(outputPathStream)) {
    outputPathStream.str(std::string());  // clears/resets the stream object to write new path
    outputPathStream << parentDirectory << prefix << nameIn << duplicateCounter << fileExtension;

    duplicateCounter++;
  }

  return outputPathStream;
}

void fileio::writeConfigFile(
  const std::string& outputFileNameIn,
  const options::MarkerDetection& detectionOptionsIn,
  const options::LineFollowerMarker& lineFollowerOptionsIn,
  const options::BoardMarkers& boardOptionsIn,
  const options::Track& trackOptionsIn,
  const options::CalibrationBoard& calibrationBoardOptionsIn,
  const options::CameraIntrinsic& cameraCalibrationParamsIn)
{
  cv::FileStorage configFile(outputFileNameIn, cv::FileStorage::WRITE);
  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  configFile.writeComment(
    "\nCommon detection parameters that apply to both detection and calibration modes.");
  configFile.startWriteStruct("marker_detection", cv::FileNode::MAP);
    configFile << "camera_id" << detectionOptionsIn.camID;
    configFile << "frame_width_pixels" << detectionOptionsIn.frameWidthPixels;
    configFile << "frame_height_pixels" << detectionOptionsIn.frameHeightPixels;
    configFile << "frame_rate_fps" << detectionOptionsIn.frameRateFPS;
    configFile << "show_rejected_markers" << detectionOptionsIn.showRejectedMarkers;
  configFile.endWriteStruct();

  configFile.writeComment("\nParameters specific to line follower marker.");
  configFile.startWriteStruct("line_follower_marker", cv::FileNode::MAP);
    configFile << "marker_side_meters" << lineFollowerOptionsIn.markerSideMeters;
    configFile << "marker_id" << lineFollowerOptionsIn.markerID;
    configFile << "marker_dictionary_id" << lineFollowerOptionsIn.markerDictionaryID;
  configFile.endWriteStruct();

  configFile.writeComment("\nParameters specific to board markers.");
  configFile.startWriteStruct("board_markers", cv::FileNode::MAP);
    configFile << "marker_side_meters" << boardOptionsIn.markerSideMeters;
    configFile << "marker_seperation_meters_x" << boardOptionsIn.markerSeperationMetersX;
    configFile << "marker_seperation_meters_y" << boardOptionsIn.markerSeperationMetersY;
    configFile << "marker_ids" << boardOptionsIn.markerIDs;
    configFile << "marker_dictionary_id" << boardOptionsIn.markerDictionaryID;
  configFile.endWriteStruct();

  configFile.writeComment("\nParameters specific to track specifications.");
  configFile << "track_selection" << static_cast<int>(trackOptionsIn.selection);

  configFile.writeComment("");
  configFile.startWriteStruct("line_track", cv::FileNode::MAP);

    configFile.startWriteStruct("point1", cv::FileNode::MAP);
      configFile << "x_meters" << trackOptionsIn.lineTrack.getPoint1().x;
      configFile << "y_meters" << trackOptionsIn.lineTrack.getPoint1().y;
    configFile.endWriteStruct();

    configFile.startWriteStruct("point2", cv::FileNode::MAP);
      configFile << "x_meters" << trackOptionsIn.lineTrack.getPoint2().x;
      configFile << "y_meters" << trackOptionsIn.lineTrack.getPoint2().y;
    configFile.endWriteStruct();

  configFile.endWriteStruct();

  configFile.writeComment("");
  configFile.startWriteStruct("round_track", cv::FileNode::MAP);

    configFile.startWriteStruct("center", cv::FileNode::MAP);
      configFile << "x_meters" << trackOptionsIn.roundTrack.getCenter().x;
      configFile << "y_meters" << trackOptionsIn.roundTrack.getCenter().y;
    configFile.endWriteStruct();

    configFile << "width_meters" << trackOptionsIn.roundTrack.getMajorAxisLength();
    configFile << "height_meters" << trackOptionsIn.roundTrack.getMinorAxisLength();

  configFile.endWriteStruct();

  configFile.writeComment("\nParameters specific to camera calibration.");
  configFile.startWriteStruct("camera_calibration", cv::FileNode::MAP);
    configFile << "marker_side_meters" << calibrationBoardOptionsIn.markerSideMeters;
    configFile << "square_side_meters" << calibrationBoardOptionsIn.squareSideMeters;
    configFile << "squares_quantity_x" << calibrationBoardOptionsIn.squaresQuantityX;
    configFile << "squares_quantity_y" << calibrationBoardOptionsIn.squaresQuantityY;
    configFile << "marker_dictionary_id" << calibrationBoardOptionsIn.markerDictionaryID;
    configFile << "camera_matrix" << cameraCalibrationParamsIn.cameraMatrix;
    configFile << "distortion_coefficients" << cameraCalibrationParamsIn.distortionCoefficients;
  configFile.endWriteStruct();

  configFile.release();
}
