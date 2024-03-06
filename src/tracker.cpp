#include "tracker.hpp"

void tracker::trackLineFollower(const trackerOptions& options)
{
  cv::VideoCapture inputVideo;
  int waitTime;
  if(!options.videoPath.empty()) {
    inputVideo.open(options.videoPath);
    waitTime = 0;
  } else {
    inputVideo.open(options.camID);
    waitTime = 10;
  }

  bool estimatePose {false};
  if (!options.camMatrix.empty() && !options.distCoeffs.empty())
    estimatePose = true;

  double totalTime = 0;
  int totalIterations = 0;

  // Set coordinate system
  cv::Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-options.markerLengthMeters/2.f, options.markerLengthMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(options.markerLengthMeters/2.f, options.markerLengthMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(options.markerLengthMeters/2.f, -options.markerLengthMeters/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-options.markerLengthMeters/2.f, -options.markerLengthMeters/2.f, 0);

  std::cout << "Hit ESC key or Crtl + C to exit if a window opens." << std::endl;

  cv::aruco::ArucoDetector detector(options.arucoDictionary, options.detectorParameters);

  while(inputVideo.grab()) {
    cv::Mat image, imageCopy, resizedImage;
    inputVideo.retrieve(image);

    cv::resize(image, resizedImage, cv::Size(426, 240));

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
        solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
      }
    }

    double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if(totalIterations % 30 == 0) {
      cout << "Detection Time = " << currentTime * 1000 << " ms "
           << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
    }

    // draw results
    resizedImage.copyTo(imageCopy);
    if(!ids.empty()) {
      aruco::drawDetectedMarkers(imageCopy, corners, ids);

      if(estimatePose) {
        for(unsigned int i = 0; i < ids.size(); i++)
          cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
      }
    }

    if(!rejected.empty())
      aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

    imshow("out", imageCopy);
    char key = (char)waitKey(waitTime);
    if(key == 27) break;
  }
}
