/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#pragma once

#include <iostream>
#include <fstream>
#include <ctime>

#include "options.hpp"

namespace tracker {

struct detectionOptions {
  detectionOptions()
  : camID {0},
    inputFilePath {"none"},
    markerSideMeters {0},
    showRejectedMarkers {false},
    detectorParameters {cv::aruco::DetectorParameters()},
    arucoDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL}
    {}

  int camID;
  cv::String inputFilePath;
  cv::Mat camMatrix;
  cv::Mat distCoeffs;
  float markerSideMeters;
  bool showRejectedMarkers;
  cv::aruco::DetectorParameters detectorParameters;
  int arucoDictionaryID;
};

struct boardOptions {
  boardOptions()
  : markerSideMeters {0},
    markerSeperationMetersX {0},
    markerSeperationMetersY {0},
    markerDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL},
    markerIDs {}
  {}

  float markerSideMeters;
  float markerSeperationMetersX;
  float markerSeperationMetersY;
  int markerDictionaryID;
  std::vector<int> markerIDs;
};

struct calibrationOptions {
  calibrationOptions()
  : camID {0},
    inputFilePath {"none"},
    markerSideMeters {0},
    squareSideMeters {0},
    squaresQuantityX {0},
    squaresQuantityY {0},
    arucoDictionaryID {cv::aruco::DICT_ARUCO_ORIGINAL}
    {}

  int camID;
  cv::String inputFilePath;
  float markerSideMeters;
  float squareSideMeters;
  int squaresQuantityX;
  int squaresQuantityY;
  int arucoDictionaryID;
};

struct calibrationOutput {
  calibrationOutput()
  : cameraMatrix {cv::Mat::zeros(cv::Size(3, 3), CV_32F)},
    distCoeffs {cv::Mat::zeros(cv::Size(5, 1), CV_32F)}
    {}

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
};

// TODO: Define error codes and implement error handling
enum class Error {
  CANNOT_OPEN_FILE = 401,
  INCOMPLETE_INFORMATION = 402
};

void generateLineFollowerBoardPoints(
  const boardOptions& board_options,
  std::vector<std::vector<cv::Point3f>>& output_board_obj_points);

void readConfigFile(const std::string& filename, detectionOptions& options);
void readConfigFile(const std::string& filename, boardOptions& options);
void readConfigFile(const std::string& filename, calibrationOptions& options);

void writeConfigFile(
  const std::string& filename,
  const detectionOptions& detection_options,
  const boardOptions& board_options,
  const calibrationOptions& calibration_options,
  const calibrationOutput& calibration_output);

void writePoseToCSV(
  std::ofstream& csv_file,
  const cv::Vec3d& tvec,
  const cv::Vec3d& rvec);

std::stringstream createTimeStampedFileName(
  const std::string& filedir,
  const std::string& prefix,
  const std::string& extension);

bool isNonZeroMatrix(const cv::Mat& matrix);

void trackLineFollower(
  const options::MarkerDetection& detectionOptions,
  const options::BoardMarkers& boardMarkersOptions,
  const options::LineFollowerMarker& lineFollowerOptions,
  const std::string& outputFileName = "none");

void calibrateCamera(const options::Calibration& options, const options::CalibrationOutput& output);
}
