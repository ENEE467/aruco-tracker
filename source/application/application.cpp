#include <filesystem>

#include <nfd.hpp>

#include "App.h"
#include "Native.h"

#include "options/Tracking.hpp"
#include "options/Calibration.hpp"
#include "tracking/Tracker/Tracker.hpp"
#include "calibration/Calibrator/Calibrator.hpp"
#include "fileio/ConfigFile/ConfigFile.hpp"

struct InterfaceWindow : App {

// using App::App;
InterfaceWindow(std::string title, int w, int h, int argc, char const *argv[]);

public:
  void Update() override;

private:
  bool _isConfigFileSet {false};
  bool _isOutputDirSet {false};
  bool _isSetupDone {false};
  bool _isCalibrationDone {false};

  std::string _configFilePath;
  std::string _outputDir;
  std::string _outputName;

  char _configFilePathCStr[128] {};
  char _outputDirCStr[128] {};
  char _outputNameCStr[128] {};

  const char* _programModes[2] = {"Tracking", "Calibration"};
  int _modeChoice {0};

  // options::Tracking _trackingOptions;
  // options::Calibration _calibrationOptions;

  cv::VideoCapture _videoObject;
  std::unique_ptr<fileio::ConfigFile> _configFile;

  std::unique_ptr<tracking::Tracker> _tracker;
  std::unique_ptr<tracking::Output> _trackingOutput;

  std::unique_ptr<calibration::Calibrator> _calibrator;
  options::Calibration _newCalibrationOptions;

  unsigned int _imageTextureToDisplay {};

  void openStartupMenu();
  void startInterface();
  void runTracker();
  void runCalibrator();

};

InterfaceWindow::InterfaceWindow(std::string title, int w, int h, int argc, char const *argv[])
: App(title, w, h, argc, argv)
{}

void InterfaceWindow::openStartupMenu()
{
  ImGui::OpenPopup("Welcome!");
  ImGui::SetNextWindowPos(
    ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

  if (!ImGui::BeginPopupModal("Welcome!"))
    return;

  // Config file input section ---------------------------------------------------------------------
  ImGui::SeparatorText("Locate the configuration file");

  bool configTextBoxInput = ImGui::InputTextWithHint(
    "##configTextBox",
    "Press ENTER to set or ESC to clear.",
    _configFilePathCStr,
    IM_ARRAYSIZE(_configFilePathCStr),
    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll);

  if (configTextBoxInput)
    _configFilePath = _configFilePathCStr;

  ImGui::SameLine();

  bool configFileExplorerInput {false};
  if (ImGui::Button("Open File Explorer##configFileExplorer")) {
    _configFilePath.clear();
    auto openDialogStatus {
      OpenDialog(_configFilePath, {{"YAML File", "yaml, YAML, yml"}}, "config/default.yaml")};

    configFileExplorerInput = (openDialogStatus == DialogOkay);

    if (configFileExplorerInput) {
      configTextBoxInput = false;
      memset(_configFilePathCStr, 0, sizeof(_configFilePathCStr));
      _configFilePath.copy(_configFilePathCStr, 128, 0);
    }
    else {
      _configFilePath = _configFilePathCStr;
    }
  }

  if (configTextBoxInput || configFileExplorerInput) {
    try {
      // fileio::readConfigFile(_configFilePath, _trackingOptions);
      // fileio::readConfigFile(_configFilePath, _calibrationOptions);

      _configFile = std::make_unique<fileio::ConfigFile>(_configFilePath);
      _isConfigFileSet = true;
    }
    catch (std::exception& exception) {
      ImGui::OpenPopup("Configuration File Read Error!");
      std::cout << exception.what() << '\n';
      _configFilePath.clear();
      memset(_configFilePathCStr, 0, sizeof(_configFilePathCStr));
      _isConfigFileSet = false;
    }
  }

  if (ImGui::BeginPopupModal("Configuration File Read Error!")) {
    ImGui::Text(
      "Please check if the file given exists, correctly formatted and has no missing information.");

    if (ImGui::Button("Dismiss", ImVec2(120, 0)))
      ImGui::CloseCurrentPopup();

    ImGui::EndPopup();
  }

  // Output directory section ----------------------------------------------------------------------
  ImGui::SeparatorText("Choose an output directory");

  bool outputTextBoxInput = ImGui::InputTextWithHint(
    "##outputTextBox",
    "Press ENTER to set or ESC to clear.",
    _outputDirCStr,
    IM_ARRAYSIZE(_outputDirCStr),
    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll);

  if (outputTextBoxInput)
    _outputDir = _outputDirCStr;

  ImGui::SameLine();

  if (ImGui::Button("Open File Explorer##outputFileExplorer")) {
    _outputDir.clear();
    auto pickDialogStatus {PickDialog(_outputDir)};

    if (pickDialogStatus == DialogOkay) {
      outputTextBoxInput = false;
      memset(_outputDirCStr, 0, sizeof(_outputDirCStr));
      _outputDir.copy(_outputDirCStr, IM_ARRAYSIZE(_outputDirCStr), 0);
    }
    else {
      _outputDir = _outputDirCStr;
    }
  }

  if (outputTextBoxInput && !std::filesystem::is_directory(_outputDir)) {
    ImGui::OpenPopup("Output Directory Error!");
    std::cout << "Given output directory does not exist." << '\n';
    _outputDir.clear();
    memset(_outputDirCStr, 0, sizeof(_outputDirCStr));
    _isOutputDirSet = false;
  }
  else {
    _isOutputDirSet = true;
  }

  if (ImGui::BeginPopupModal("Output Directory Error!")) {
    ImGui::Text("Given output directory does not exist.");

    if (ImGui::Button("Dismiss", ImVec2(120, 0)));
      ImGui::CloseCurrentPopup();

    ImGui::EndPopup();
  }

  // Output name section ---------------------------------------------------------------------------
  ImGui::SeparatorText("Name your run");

  bool nameTextBoxInput = ImGui::InputTextWithHint(
    "##outputNameTextBox",
    "Press ENTER to set or ESC to clear",
    _outputNameCStr,
    IM_ARRAYSIZE(_outputNameCStr),
    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll);

  // if (nameTextBoxInput) {
  //   _outputName = _outputNameCStr;
  //   // _isOutputNameSet = true;
  // }

  _outputName = _outputNameCStr;

  if (_outputName.empty()) {
    memset(_outputNameCStr, 0, sizeof(_outputNameCStr));
    _outputName = fileio::createTimeStamp();
    _outputName.copy(_outputNameCStr, IM_ARRAYSIZE(_outputNameCStr), 0);
  }

  // Program mode selection section ----------------------------------------------------------------
  ImGui::SeparatorText("Choose the program mode");
  ImGui::Combo("##", &_modeChoice, _programModes, IM_ARRAYSIZE(_programModes));

  ImGui::Dummy(ImVec2(0.0, 10.0));

  // Start program button --------------------------------------------------------------------------
  if (ImGui::Button("Start Program##startProgram") && _isConfigFileSet && _isOutputDirSet) {
    switch (_modeChoice) {

    case 0:
      _tracker = std::make_unique<tracking::Tracker>(_configFile->getTrackingOptions());
      _trackingOutput = std::make_unique<tracking::Output>(_outputDir, _outputName);
      _isSetupDone = true;
      break;

    case 1:
      _calibrator = std::make_unique<calibration::Calibrator>(_configFile->getCalibrationOptions());
      _isSetupDone = true;
      break;

    default:
      break;

    }

    if (_isSetupDone) {
      const auto& detectionOptions {_configFile->getTrackingOptions().detection};

      _videoObject.open(detectionOptions.camID);
      _videoObject.set(cv::CAP_PROP_FRAME_WIDTH, detectionOptions.frameWidthPixels);
      _videoObject.set(cv::CAP_PROP_FRAME_HEIGHT, detectionOptions.frameHeightPixels);
      _videoObject.set(cv::CAP_PROP_FPS, detectionOptions.frameRateFPS);

      ImGui::CloseCurrentPopup();
    }
  }

  ImGui::EndPopup();
}

void InterfaceWindow::runTracker()
{
  _tracker->run(_videoObject, _imageTextureToDisplay);
  _tracker->writeOutput(*_trackingOutput);

  for (ImGuiKey key = ImGuiKey_NamedKey_BEGIN; key < ImGuiKey_NamedKey_END; key = (ImGuiKey)(key + 1)) {
    if (false || !ImGui::IsKeyDown(key))
      continue;

    if (key != ImGuiKey_Space)
      continue;

    if (!_trackingOutput->isOpen())
      _trackingOutput->open(_configFile->getTrackingOptions());
    else
      _trackingOutput->close();
  }
}

void InterfaceWindow::runCalibrator()
{
  // _calibrator->run(_videoObject, _imageTextureToDisplay);

  // if (_isCalibrationDone)
  //   return;

  // for (ImGuiKey key = ImGuiKey_NamedKey_BEGIN; key < ImGuiKey_NamedKey_END; key = (ImGuiKey)(key + 1)) {
  //   if (false || !ImGui::IsKeyDown(key))
  //     continue;

  //   switch (key) {

  //   case ImGuiKey_C:
  //     _calibrator->captureFrame();
  //     break;

  //   case ImGuiKey_Escape:
  //     _newCalibrationOptions = _configFile->getCalibrationOptions();
  //     _isCalibrationDone = _calibrator->finishCalibration(_newCalibrationOptions.intrinsicParams);
  //     _configFile->setCalibrationOptions(_newCalibrationOptions);
  //     _configFile->saveFile();
  //     break;

  //   default:
  //     break;

  //   }
  // }
}

void InterfaceWindow::Update()
{
  ImGui::SetNextWindowSize(GetWindowSize());
  ImGui::SetNextWindowPos({0, 0});

  bool windowStarted = ImGui::Begin("Line Follower Tracker", nullptr,
    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);

  if (!windowStarted) {
    ImGui::End();
    return;
  }

  if (!_isSetupDone) {
    openStartupMenu();
  }
  else {
    switch (_modeChoice) {

    case 0:
      runTracker();
      break;

    case 1:
      runCalibrator();
      break;

    default:
      break;

    }
  }

  if (_isSetupDone) {
    ImGui::Image(
      (void*)(intptr_t)_imageTextureToDisplay,
      ImVec2(
        _configFile->getTrackingOptions().detection.frameWidthPixels,
        _configFile->getTrackingOptions().detection.frameHeightPixels));
  }



  // ImGui::ShowDemoWindow();

  ImGui::End();
}

int main(int argc, char const *argv[])
{
  InterfaceWindow testInterface {"Line Follower Tracker", 640, 480, argc, argv};
  testInterface.Run();

  return 0;
}
