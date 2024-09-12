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
  cv::Mat _frame;

  std::chrono::_V2::system_clock::time_point _currentTime {std::chrono::high_resolution_clock::now()};
  std::chrono::_V2::system_clock::time_point _timeAtSpacePressed {std::chrono::high_resolution_clock::now()};
  std::chrono::duration<double, std::ratio<1L, 1L>> _durationSinceSpacePressed {};

  void openStartupMenu();
  void startInterface();
  void runTracker();
  void runCalibrator();
  void displayCVFrame(cv::Mat& frameIn);

};

InterfaceWindow::InterfaceWindow(std::string title, int w, int h, int argc, char const *argv[])
: App(title, w, h, argc, argv)
{
  glGenTextures(1, &_imageTextureToDisplay);
}

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

  // Generate empty configuration file button
  ImGui::SameLine();
  // ImGui::Dummy(ImVec2(10.0, 0.0));
  if (ImGui::Button("Generate Empty Configuration File##genEmptyConfig") && _isOutputDirSet) {
    fileio::ConfigFile emptyConfigFile {_outputDir, _outputName, true};
    emptyConfigFile.saveFile();
  }

  ImGui::EndPopup();
}

void InterfaceWindow::displayCVFrame(cv::Mat& frameIn)
{
  GLenum minFilter {GL_LINEAR_MIPMAP_LINEAR};
  GLenum magFilter {GL_LINEAR};
  GLenum wrapFilter {GL_CLAMP_TO_EDGE};
  // Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, _imageTextureToDisplay);

	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
		magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		// cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}

	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_BGR;
	if (frameIn.channels() == 1)
	{
		inputColourFormat = GL_RED;
	}

	// Create the texture
	glTexImage2D(
    GL_TEXTURE_2D,     // Type of texture
		0,                 // Pyramid level (for mip-mapping) - 0 is the top level
		GL_RGB,            // Internal colour format to convert to
		frameIn.cols,          // Image width  i.e. 640 for Kinect in standard mode
		frameIn.rows,          // Image height i.e. 480 for Kinect in standard mode
		0,                 // Border width in pixels (can either be 1 or 0)
		inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
		GL_UNSIGNED_BYTE,  // Image data type
		frameIn.ptr());        // The actual image data itself

  // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
	if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
		minFilter == GL_LINEAR_MIPMAP_NEAREST ||
		minFilter == GL_NEAREST_MIPMAP_LINEAR ||
		minFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		glGenerateMipmap(GL_TEXTURE_2D);
	}

  ImGui::Image(
    (void*)(intptr_t)_imageTextureToDisplay,
    ImVec2(
      _configFile->getTrackingOptions().detection.frameWidthPixels,
      _configFile->getTrackingOptions().detection.frameHeightPixels));
}

void InterfaceWindow::runTracker()
{
  _tracker->run(_videoObject, _frame);
  _tracker->writeOutput(*_trackingOutput);

  _durationSinceSpacePressed = _currentTime - _timeAtSpacePressed;

  for (ImGuiKey key = ImGuiKey_NamedKey_BEGIN; key < ImGuiKey_NamedKey_END; key = (ImGuiKey)(key + 1)) {
    if (!ImGui::IsKeyDown(key))
      continue;

    if (key != ImGuiKey_Space)
      continue;

    if (_durationSinceSpacePressed.count() < 0.5)
      continue;

    _timeAtSpacePressed = _currentTime;

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
  _currentTime = std::chrono::high_resolution_clock::now();

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

  if (_isSetupDone)
    displayCVFrame(_frame);



  // ImGui::ShowDemoWindow();

  ImGui::End();
}

int main(int argc, char const *argv[])
{
  InterfaceWindow testInterface {"Line Follower Tracker", 640, 480, argc, argv};
  testInterface.Run();

  return 0;
}
