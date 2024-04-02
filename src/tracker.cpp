#include <iomanip>

#include "tracker.hpp"

void tracker::generateLineFollowerBoardPoints(
  const boardOptions& boardOptions,
  std::vector<std::vector<cv::Point3f>>& outputBoardPoints)
{
  std::vector<cv::Point3f> markerObjPoints {
    cv::Point3f(-boardOptions.markerSideMeters/2.f,
                 boardOptions.markerSideMeters/2.f,
                 0),

    cv::Point3f(boardOptions.markerSideMeters/2.f,
                boardOptions.markerSideMeters/2.f,
                0),

    cv::Point3f(boardOptions.markerSideMeters/2.f,
                -boardOptions.markerSideMeters/2.f,
                0),

    cv::Point3f(-boardOptions.markerSideMeters/2.f,
                -boardOptions.markerSideMeters/2.f,
                0)
  };

  auto translateMarkerObjpoints = [markerObjPoints]
                                  (float xDisplacement, float yDisplacement)
  {
    std::vector<cv::Point3f> translatedPoints {};

    for (const auto& markerObjPoint: markerObjPoints) {
      translatedPoints.push_back(
        markerObjPoint + cv::Point3f(xDisplacement, yDisplacement, 0));
    }

    return translatedPoints;
  };

  outputBoardPoints.push_back(
    translateMarkerObjpoints(0, boardOptions.markerSeperationMetersY));

  outputBoardPoints.push_back(
    translateMarkerObjpoints(boardOptions.markerSeperationMetersX,
                             boardOptions.markerSeperationMetersY));

  outputBoardPoints.push_back(
    translateMarkerObjpoints(boardOptions.markerSeperationMetersX, 0));

  outputBoardPoints.push_back(translateMarkerObjpoints(0, 0));
}

void tracker::readConfigFile(const std::string& filename, tracker::detectionOptions& options)
{
  cv::FileStorage configFile(filename, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto markerDetectionNode {configFile["marker_detection"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  auto camIDNode {markerDetectionNode["camera_id"]};
  auto inputFileNode {markerDetectionNode["input_source_path"]};
  auto markerSideNode {markerDetectionNode["marker_side_meters"]};
  auto dictionaryIDNode {markerDetectionNode["marker_dictionary"]};
  auto cameraCalibrationNode {configFile["camera_calibration"]};

  cv::read(camIDNode, options.camID, 0);
  cv::read(inputFileNode, options.inputFilePath, "");
  cv::read(markerSideNode, options.markerSideMeters, 0.1);
  cv::read(dictionaryIDNode, options.arucoDictionaryID, cv::aruco::DICT_ARUCO_ORIGINAL);

  // read camera calibration parameters
  if (!cameraCalibrationNode.empty() && cameraCalibrationNode.isMap()) {
    auto cameraMatrixNode {cameraCalibrationNode["camera_matrix"]};
    auto distortionCoefficientNode {cameraCalibrationNode["distortion_coefficients"]};

    cv::read(cameraMatrixNode, options.camMatrix);
    cv::read(distortionCoefficientNode, options.distCoeffs);
  }
}

void tracker::readConfigFile(const std::string& filename, tracker::boardOptions& options)
{
  cv::FileStorage configFile(filename, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto markerDetectionNode {configFile["marker_detection"]};
  auto boardParametersNode {configFile["board_markers"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  if (boardParametersNode.empty() || !boardParametersNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  auto markerSideMetersNode {boardParametersNode["marker_side_meters"]};
  auto markerSeperationMetersXNode {boardParametersNode["marker_seperation_meters_x"]};
  auto markerSeperationMetersYNode {boardParametersNode["marker_seperation_meters_y"]};
  auto dictionaryIDNode {markerDetectionNode["marker_dictionary"]};
  auto markerIDsNode {boardParametersNode["marker_ids"]};

  cv::read(markerSideMetersNode, options.markerSideMeters, 0.F);
  cv::read(markerSeperationMetersXNode, options.markerSeperationMetersX, 0.F);
  cv::read(markerSeperationMetersYNode, options.markerSeperationMetersY, 0.F);
  cv::read(dictionaryIDNode, options.markerDictionaryID, cv::aruco::DICT_ARUCO_ORIGINAL);
  cv::read(markerIDsNode, options.markerIDs, {});
}

void tracker::readConfigFile(const std::string& filename, tracker::calibrationOptions& options)
{
  cv::FileStorage configFile(filename, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto markerDetectionNode {configFile["marker_detection"]};
  auto cameraCalibrationNode {configFile["camera_calibration"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  if (cameraCalibrationNode.empty() || !cameraCalibrationNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  auto camIDNode {markerDetectionNode["camera_id"]};
  auto inputFileNode {markerDetectionNode["input_source_path"]};
  auto markerSideNode {markerDetectionNode["marker_side_meters"]};
  auto dictionaryIDNode {markerDetectionNode["marker_dictionary"]};
  auto squareSideNode {cameraCalibrationNode["square_side_meters"]};
  auto squaresQuantityXNode {cameraCalibrationNode["squares_quantity_x"]};
  auto squaresQuantityYNode {cameraCalibrationNode["squares_quantity_y"]};

  cv::read(camIDNode, options.camID, 0);
  cv::read(inputFileNode, options.inputFilePath, "");
  cv::read(markerSideNode, options.markerSideMeters, 0.F);
  cv::read(squareSideNode, options.squareSideMeters, 0.F);
  cv::read(squaresQuantityXNode, options.squaresQuantityX, 0);
  cv::read(squaresQuantityYNode, options.squaresQuantityY, 0);
  cv::read(dictionaryIDNode, options.arucoDictionaryID, cv::aruco::DICT_ARUCO_ORIGINAL);
}

std::stringstream tracker::createTimeStampedFileName(
  const std::string& filedir,
  const std::string& prefix,
  const std::string& extension)
{
  std::stringstream filenameStream;
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  filenameStream << filedir << prefix << "-"
                 << std::put_time(&tm, "%Y%m%d-%H%M%S")
                 << "." << extension;

  return filenameStream;
}

void tracker::writeConfigFile(
  const std::string& filename,
  const detectionOptions& detection_options,
  const boardOptions& board_options,
  const calibrationOptions& calibration_options,
  const calibrationOutput& calibration_output)
{
  cv::FileStorage configFile(filename, cv::FileStorage::WRITE);
  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  configFile.startWriteStruct("marker_detection", cv::FileNode::MAP);
    configFile << "input_source_path" << detection_options.inputFilePath;
    configFile << "camera_id" << detection_options.camID;
    configFile << "marker_dictionary" << detection_options.arucoDictionaryID;
    configFile << "marker_side_meters" << detection_options.markerSideMeters;
  configFile.endWriteStruct();

  configFile.startWriteStruct("board_markers", cv::FileNode::MAP);
    configFile << "marker_side_meters" << board_options.markerSideMeters;
    configFile << "marker_seperation_meters_x" << board_options.markerSeperationMetersX;
    configFile << "marker_seperation_meters_y" << board_options.markerSeperationMetersY;
    configFile << "marker_dictionary_id" << board_options.markerDictionaryID;
    configFile << "marker_ids" << board_options.markerIDs;
  configFile.endWriteStruct();

  configFile.startWriteStruct("camera_calibration", cv::FileNode::MAP);
    configFile << "squares_quantity_x" << calibration_options.squaresQuantityX;
    configFile << "squares_quantity_y" << calibration_options.squaresQuantityY;
    configFile << "square_side_meters" << calibration_options.squareSideMeters;
    configFile << "camera_matrix" << calibration_output.cameraMatrix;
    configFile << "distortion_coefficients" << calibration_output.distCoeffs;
  configFile.endWriteStruct();

  configFile.release();
}

void tracker::writePoseToCSV(
  std::ofstream& csv_file,
  const cv::Vec3d& tvec,
  const cv::Vec3d& rvec)
{
  csv_file << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << ", "
           << rvec[0] << ", " << rvec[1] << ", " << rvec[2] << std::endl;
}

void tracker::trackLineFollower(
  const tracker::detectionOptions& detectionOptions,
  const boardOptions& boardOptions,
  const std::string& outputFileName)
{
  bool hasOutputFile {outputFileName != "none"};
  std::ofstream posesOutputFile;

  if (hasOutputFile) {
    posesOutputFile = std::ofstream(outputFileName);
    if (!posesOutputFile.is_open())
      throw Error::CANNOT_OPEN_FILE;
  }
  else {
    std::cout << "No output directory given, poses will not be recorded."
              << std::endl;
  }

  cv::VideoCapture inputVideo;
  int waitTime;

  if(detectionOptions.inputFilePath != "none") {
    inputVideo.open(detectionOptions.inputFilePath);
    std::cout << "Using input source file instead of camera stream" << std::endl;
    waitTime = 0;
  }
  else {
    inputVideo.open(detectionOptions.camID);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 30);
    waitTime = 10;
  }

  bool estimatePose {false};

  cv::Mat zero_cam_matrix {
    cv::Mat::zeros(
      detectionOptions.camMatrix.rows,
      detectionOptions.camMatrix.cols,
      detectionOptions.camMatrix.type()
    )
  };
  cv::Mat cam_matrix_comp {detectionOptions.camMatrix != zero_cam_matrix};

  cv::Mat zero_dist_coeff {
    cv::Mat::zeros(
      detectionOptions.distCoeffs.rows,
      detectionOptions.distCoeffs.cols,
      detectionOptions.camMatrix.type()
    )
  };
  cv::Mat dist_coeff_comp {detectionOptions.distCoeffs != zero_dist_coeff};

  if (cv::countNonZero(cam_matrix_comp) && cv::countNonZero(dist_coeff_comp)) {
    std::cout << "Marker pose estimation is active" << std::endl;
    estimatePose = true;
  }

  double totalTime = 0;
  int totalIterations = 0;

  auto arucoDictionary {cv::aruco::getPredefinedDictionary(detectionOptions.arucoDictionaryID)};

  std::vector<std::vector<cv::Point3f>> lineFollowerBoardPoints {};
  tracker::generateLineFollowerBoardPoints(
    boardOptions, lineFollowerBoardPoints);

  cv::aruco::Board lineFollowerBoard(
    lineFollowerBoardPoints, arucoDictionary, boardOptions.markerIDs);

  cv::aruco::ArucoDetector detector(arucoDictionary, detectionOptions.detectorParameters);

  std::cout << "Hit ESC key or Crtl + C to exit if a window opens." << std::endl;

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);

    double tick = (double)cv::getTickCount();

    std::vector<int> ids {};
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // TODO: Make the board detection robust.
    // detect markers and estimate pose
    detector.detectMarkers(image, corners, ids, rejected);

    cv::Vec3d tvec, rvec;

    // TODO: Board pose should be estimated only after it's properly detected.
    if(estimatePose && !ids.empty()) {
      cv::Mat objPoints, imgPoints;
      lineFollowerBoard.matchImagePoints(corners, ids, objPoints, imgPoints);
      cv::solvePnP(objPoints, imgPoints, detectionOptions.camMatrix,
                   detectionOptions.distCoeffs, rvec, tvec);

      // Record the poses if there's an output file
      // if (hasOutputFile)
        // tracker::writePoseToCSV(posesOutputFile, tvec, rvec);
    }

    double currentTime = ((double)cv::getTickCount() - tick) / cv::getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if(totalIterations % 30 == 0) {
      std::cout << "Detection Time = " << currentTime * 1000 << " ms "
        << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << std::endl;
    }

    // draw results
    image.copyTo(imageCopy);
    if(!ids.empty()) {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

      if(estimatePose && !ids.empty()) {
        cv::drawFrameAxes(imageCopy, detectionOptions.camMatrix, detectionOptions.distCoeffs, rvec, tvec, boardOptions.markerSideMeters * 1.5f, 2);
      }
    }

    if(detectionOptions.showRejectedMarkers == true && !rejected.empty())
      cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));

    cv::imshow("out", imageCopy);
    char key = (char)cv::waitKey(waitTime);
    if(key == 27) break;
  }
}

void tracker::calibrateCamera(const tracker::calibrationOptions& options, const tracker::calibrationOutput& output)
{
  bool showChessboardCorners {true};
  int calibrationFlags {0};
  cv::aruco::DetectorParameters detectorParams {cv::aruco::DetectorParameters()};

  cv::VideoCapture inputVideo;
  int waitTime;

  if(options.inputFilePath != "none") {
    inputVideo.open(options.inputFilePath);
    std::cout << "Using input source file instead of camera stream" << std::endl;
    waitTime = 0;
  }
  else {
    inputVideo.open(options.camID);
    inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    inputVideo.set(cv::CAP_PROP_FPS, 30);
    waitTime = 10;
  }

  auto arucoDictionary {cv::aruco::getPredefinedDictionary(options.arucoDictionaryID)};
  cv::aruco::CharucoBoard board(
    cv::Size(options.squaresQuantityX, options.squaresQuantityY),
    options.squareSideMeters,
    options.markerSideMeters,
    arucoDictionary
  );

  // Charuco and Detector parameters are currently default
  cv::aruco::CharucoParameters charucoParams;
  cv::aruco::CharucoDetector detector(board, charucoParams, detectorParams);

  std::vector<cv::Mat> allCharucoCorners;
  std::vector<cv::Mat> allCharucoIds;

  std::vector<std::vector<cv::Point2f>> allImagePoints;
  std::vector<std::vector<cv::Point3f>> allObjectPoints;

  std::vector<cv::Mat> allImages;
  cv::Size imageSize;

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy;
    inputVideo.retrieve(image);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedMarkers;
    cv::Mat currentCharucoCorners;
    cv::Mat currentCharucoIds;
    std::vector<cv::Point3f> currentObjectPoints;
    std::vector<cv::Point2f> currentImagePoints;

    // Detect ChArUco board
    detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

    // Draw results
    image.copyTo(imageCopy);
    if(!markerIds.empty()) {
      cv::aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if(currentCharucoCorners.total() > 3) {
      cv::aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
    }

    cv::putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
        cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

    cv::imshow("out", imageCopy);

    // Wait for key pressed
    char key = (char)cv::waitKey(waitTime);

    if(key == 27)
      break;

    if(key == 'c' && currentCharucoCorners.total() > 3) {
      // Match image points
      board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

      if(currentImagePoints.empty() || currentObjectPoints.empty()) {
        std::cout << "Point matching failed, try again." << std::endl;
        continue;
      }

      std::cout << "Frame captured" << std::endl;

      allCharucoCorners.push_back(currentCharucoCorners);
      allCharucoIds.push_back(currentCharucoIds);
      allImagePoints.push_back(currentImagePoints);
      allObjectPoints.push_back(currentObjectPoints);
      allImages.push_back(image);

      imageSize = image.size();
    }
  }

  if(allCharucoCorners.size() < 4) {
    std::cerr << "Not enough corners for calibration" << std::endl;
    return;
  }

  // Calibrate camera using ChArUco
  double repError = cv::calibrateCamera(
    allObjectPoints, allImagePoints, imageSize,
    output.cameraMatrix, output.distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(),
    cv::noArray(), cv::noArray(), calibrationFlags
  );
}
