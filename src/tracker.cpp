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
  int dictionaryID {};
  cv::read(dictionaryIDNode, dictionaryID, 16);
  options.arucoDictionary = cv::aruco::getPredefinedDictionary(dictionaryID);

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
    std::cout << "Camera matrix and distortion coefficients are not empty" << std::endl;
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

void tracker::calibrateCamera(const calibrationOptions& options, const calibrationOutput& output)
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

  cv::Mat cameraMatrix, distCoeffs;

  // Calibrate camera using ChArUco
  double repError = cv::calibrateCamera(
    allObjectPoints, allImagePoints, imageSize,
    cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::noArray(),
    cv::noArray(), cv::noArray(), calibrationFlags
  );
}
