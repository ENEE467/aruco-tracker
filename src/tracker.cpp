#include <iomanip>
#include <fstream>

#include "tracker.hpp"

void tracker::readConfigFile(const std::string& filename, tracker::detectionOptions& options)
{
  cv::FileStorage configFile(filename, cv::FileStorage::READ);

  if (!configFile.isOpened())
    throw Error::CANNOT_OPEN_FILE;

  auto markerDetectionNode {configFile["marker_detection"]};

  if (markerDetectionNode.empty() || !markerDetectionNode.isMap())
    throw Error::INCOMPLETE_INFORMATION;

  // read camera id
  auto camIDNode {markerDetectionNode["camera_id"]};
  read(camIDNode, options.camID, 0);

  // read input file path
  auto inputFileNode {markerDetectionNode["input_source_path"]};
  read(inputFileNode, options.inputFilePath, "");

  // read marker side length
  auto markerSideNode {markerDetectionNode["marker_side_meters"]};
  read(markerSideNode, options.markerSideMeters, 0.1);

  // read aruco dictionary option
  auto dictionaryIDNode {markerDetectionNode["marker_dictionary"]};
  cv::read(dictionaryIDNode, options.arucoDictionaryID, cv::aruco::DICT_ARUCO_ORIGINAL);

  auto cameraCalibrationNode {configFile["camera_calibration"]};

  // read camera calibration parameters
  if (!cameraCalibrationNode.empty() && cameraCalibrationNode.isMap()) {
    // read camera matrix
    auto cameraMatrixNode {cameraCalibrationNode["camera_matrix"]};
    cv::read(cameraMatrixNode, options.camMatrix);

    // read distortion coefficients
    auto distortionCoefficientNode {cameraCalibrationNode["distortion_coefficients"]};
    cv::read(distortionCoefficientNode, options.distCoeffs);
  }
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

  // read camera id
  auto camIDNode {markerDetectionNode["camera_id"]};
  read(camIDNode, options.camID, 0);

  // read input file path
  auto inputFileNode {markerDetectionNode["input_source_path"]};
  read(inputFileNode, options.inputFilePath, "");

  // marker side length
  auto markerSideNode {markerDetectionNode["marker_side_meters"]};
  read(markerSideNode, options.markerSideMeters, 0.F);

  // square side length
  auto squareSideNode {cameraCalibrationNode["square_side_meters"]};
  read(squareSideNode, options.squareSideMeters, 0.F);

  // squares quantity x
  auto squaresQuantityXNode {cameraCalibrationNode["squares_quantity_x"]};
  read(squaresQuantityXNode, options.squaresQuantityX, 0);

  // squares quantity y
  auto squaresQuantityYNode {cameraCalibrationNode["squares_quantity_y"]};
  read(squaresQuantityYNode, options.squaresQuantityY, 0);

  // read aruco dictionary option
  auto dictionaryIDNode {markerDetectionNode["marker_dictionary"]};
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

  configFile.startWriteStruct("camera_calibration", cv::FileNode::MAP);
  configFile << "squares_quantity_x" << calibration_options.squaresQuantityX;
  configFile << "squares_quantity_y" << calibration_options.squaresQuantityY;
  configFile << "square_side_meters" << calibration_options.squareSideMeters;
  configFile << "camera_matrix" << calibration_output.cameraMatrix;
  configFile << "distortion_coefficients" << calibration_output.distCoeffs;
  configFile.endWriteStruct();

  configFile.release();
}

void tracker::trackLineFollower(const tracker::detectionOptions& options)
{
  cv::VideoCapture inputVideo;
  int waitTime;

  if(options.inputFilePath != "none") {
    inputVideo.open(options.inputFilePath);
    std::cout << "Using input source file instead of camera stream" << std::endl;
    waitTime = 0;
  }
  else {
    inputVideo.open(options.camID);
    waitTime = 10;
  }

  bool estimatePose {false};

  cv::Mat zero_cam_matrix {
    cv::Mat::zeros(
      options.camMatrix.rows,
      options.camMatrix.cols,
      options.camMatrix.type()
    )
  };
  cv::Mat cam_matrix_comp {options.camMatrix != zero_cam_matrix};

  cv::Mat zero_dist_coeff {
    cv::Mat::zeros(
      options.distCoeffs.rows,
      options.distCoeffs.cols,
      options.camMatrix.type()
    )
  };
  cv::Mat dist_coeff_comp {options.distCoeffs != zero_dist_coeff};

  if (cv::countNonZero(cam_matrix_comp) && cv::countNonZero(dist_coeff_comp)) {
    std::cout << "Marker pose estimation active" << std::endl;
    estimatePose = true;
  }

  double totalTime = 0;
  int totalIterations = 0;

  // Set coordinate system
  cv::Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-options.markerSideMeters/2.f, options.markerSideMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(options.markerSideMeters/2.f, options.markerSideMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(options.markerSideMeters/2.f, -options.markerSideMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-options.markerSideMeters/2.f, -options.markerSideMeters/2.f, 0);

  std::cout << "Hit ESC key or Crtl + C to exit if a window opens." << std::endl;

  auto arucoDictionary {cv::aruco::getPredefinedDictionary(options.arucoDictionaryID)};
  cv::aruco::ArucoDetector detector(arucoDictionary, options.detectorParameters);

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy, resizedImage;
    inputVideo.retrieve(image);

    cv::resize(image, resizedImage, cv::Size(), 0.25, 0.25);

    double tick = (double)cv::getTickCount();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // detect markers and estimate pose
    detector.detectMarkers(resizedImage, corners, ids, rejected);

    size_t  nMarkers = corners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if(estimatePose && !ids.empty()) {
      // Calculate pose for each marker
      for (size_t  i = 0; i < nMarkers; i++) {
        cv::solvePnP(objPoints, corners.at(i), options.camMatrix, options.distCoeffs, rvecs.at(i), tvecs.at(i));
      }
    }

    double currentTime = ((double)cv::getTickCount() - tick) / cv::getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if(totalIterations % 30 == 0) {
      std::cout << "Detection Time = " << currentTime * 1000 << " ms "
        << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << std::endl;
    }

    // draw results
    resizedImage.copyTo(imageCopy);
    if(!ids.empty()) {
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

      if(estimatePose) {
        for(unsigned int i = 0; i < ids.size(); i++)
          cv::drawFrameAxes(imageCopy, options.camMatrix, options.distCoeffs, rvecs[i], tvecs[i], options.markerSideMeters * 1.5f, 2);
      }
    }

    if(options.showRejectedMarkers == true && !rejected.empty())
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
