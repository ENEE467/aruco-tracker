#include <filesystem>

#include <nfd.hpp>

#include "app-framework/App.h"
#include "app-framework/Native.h"
#include "core/fileio.hpp"
#include "core/options.hpp"

struct InterfaceWindow : App {

using App::App;

public:
  void Update() override;

private:
  bool _isConfigFileSet {false};
  bool _isOutputDirSet {false};
  bool _isSetupDone {false};

  std::string _configFilePath;
  std::string _outputDir;
  std::string _outputName;

  char _configFilePathCStr[128] {};
  char _outputDirCStr[128] {};
  char _outputNameCStr[128] {};

  const char* _programModes[2] = {"Tracking", "Calibration"};
  int _modeChoice {0};

  options::Tracking _trackingOptions;
  options::Calibration _calibrationOptions;

  void openStartupMenu();
  void startInterface();

};

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
      fileio::readConfigFile(_configFilePath, _trackingOptions);
      fileio::readConfigFile(_configFilePath, _calibrationOptions);
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
      "Please check if the correct file is given, correctly formatted and has no missing information.");

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
      _outputDir.copy(_outputDirCStr, 128, 0);
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
  ImGui::SeparatorText("Set an output file name");

  bool nameTextBoxInput = ImGui::InputTextWithHint(
    "##outputNameTextBox",
    "Press ENTER to set or ESC to clear",
    _outputNameCStr,
    IM_ARRAYSIZE(_outputNameCStr),
    ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_EscapeClearsAll);

  if (nameTextBoxInput)
    _outputName = _outputNameCStr;

  if (_outputName.empty()) {
    _outputName.clear();
    memset(_outputNameCStr, 0, sizeof(_outputNameCStr));
    _outputName = fileio::createTimeStamp().str();
    _outputName.copy(_outputNameCStr, 128, 0);
  }

  ImGui::SeparatorText("Choose the program mode");
  ImGui::Combo("##", &_modeChoice, _programModes, IM_ARRAYSIZE(_programModes));

  ImGui::EndPopup();
}

void InterfaceWindow::Update()
{
  ImGui::SetNextWindowSize(GetWindowSize());
  ImGui::SetNextWindowPos({0, 0});

  bool windowStarted = ImGui::Begin("Line Follower Tracker", nullptr,
    ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

  if (!windowStarted) {
    ImGui::End();
    return;
  }

  openStartupMenu();
  // ImGui::ShowDemoWindow();

  // Ask for configuration file.


  // ImGui::Text("Provide a path to the input configuration file...");

  // NFD_Init();

  // nfdu8char_t *outPath;
  // nfdu8filteritem_t filters[2] = { { "Configuration file", "yaml,YAML,yml" }};
  // nfdopendialogu8args_t args = {0};
  // args.filterList = filters;
  // args.filterCount = 1;
  // nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
  // if (result == NFD_OKAY)
  // {
  //     std::cout << "Success!" << '\n';
  //     std::cout << outPath << '\n';
  //     NFD_FreePathU8(outPath);
  // }
  // else if (result == NFD_CANCEL)
  // {
  //     std::cout << "User pressed cancel." << '\n';
  // }
  // else
  // {
  //     std::cout << "Error: " << NFD_GetError() << '\n';
  // }

  ImGui::End();
}

int main(int argc, char const *argv[])
{
  InterfaceWindow testInterface {"Line Follower Tracker", 640, 480, argc, argv};
  testInterface.Run();

  return 0;
}
