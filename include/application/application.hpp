#pragma once

#include <nfd.hpp>
#include "app-framework/App.h"
#include "core/options.hpp"

struct InterfaceWindow : App {

using App::App;

public:
  void Start() override;
  void Update() override;

private:
  bool _isStartupDone;
  bool _isCalibrationMode;

  std::string _configFilePath;
  std::string _outputDir;
  std::string _outputName;

  options::Tracking _trackingOptions;
  options::Calibration _calibrationOptions;

  void openStartupWindow();
  void startInterface();

};
