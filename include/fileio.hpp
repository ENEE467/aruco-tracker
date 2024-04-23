#pragma once

#include <string>

#include "options.hpp"

namespace fileio {

void readConfigFile(const std::string& filename, options::MarkerDetection& options);
void readConfigFile(const std::string& filename, options::LineFollowerMarker& options);
void readConfigFile(const std::string& filename, options::BoardMarkers& options);
void readConfigFile(const std::string& filename, options::Calibration& options);

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
