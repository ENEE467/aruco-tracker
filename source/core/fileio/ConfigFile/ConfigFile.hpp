#pragma once

#include <opencv2/core/persistence.hpp>

#include "File/File.hpp"
#include "Tracking.hpp"
#include "Calibration.hpp"

namespace fileio {

class ConfigFile : public File {

public:
  ConfigFile(
    const std::string& configDirectoryIn,
    const std::string& nameIn,
    bool isTemplateFileIn = "false");

  ConfigFile(const std::string& filePathIn);

  const options::Tracking& getTrackingOptions() {return _trackingOptions;}
  const options::Calibration& getCalibrationOptions() {return _calibrationOptions;}
  bool isOpen() const override {return _cvFileObject.isOpened();}

  void setTrackingOptions(options::Tracking& trackingOptionsIn)
  {
    _trackingOptions = std::move(trackingOptionsIn);
  }

  void setCalibrationOptions(const options::Calibration& calibrationOptionsIn)
  {
    _calibrationOptions = calibrationOptionsIn;
  }

  void saveFile();


private:
  cv::FileStorage _cvFileObject;
  bool _isTemplateFile;

  options::Tracking _trackingOptions;
  options::Calibration _calibrationOptions;

};

}
