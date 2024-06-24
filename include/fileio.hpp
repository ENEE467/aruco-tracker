#pragma once

#include <string>

#include "options.hpp"

namespace fileio {

void readConfigFile(const std::string& filenameIn, options::MarkerDetection& optionsOut);
void readConfigFile(const std::string& filenameIn, options::LineFollowerMarker& optionsOut);
void readConfigFile(const std::string& filenameIn, options::BoardMarkers& optionsOut);
void readConfigFile(const std::string& filenameIn, options::Track& optionsOut);
void readConfigFile(const std::string& filenameIn, options::CalibrationBoard& optionsOut);
void readConfigFile(const std::string& filenameIn, options::CameraIntrinsic& parametersOut);

void readConfigFile(const std::string& filenameIn, options::Tracking& optionsOut);
void readConfigFile(const std::string& filenameIn, options::Calibration& optionsOut);

void writeConfigFile(
  const std::string& filename,
  const options::MarkerDetection& detection_options,
  const options::LineFollowerMarker& line_follower_options,
  const options::BoardMarkers& board_options,
  const options::Calibration& calibration_options,
  const options::CalibrationOutput& calibration_output);

void writePoseToCSV(
  std::ofstream& csv_file,
  const cv::Vec3d& tvec,
  const cv::Vec3d& rvec);

std::stringstream createTimeStampedFileName(
  const std::string& filedir,
  const std::string& prefix,
  const std::string& extension);

}
