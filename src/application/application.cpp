#include "application/application.hpp"

struct InterfaceWindow : App {

  using App::App;

  void Update() override
  {
    ImGui::SetNextWindowSize(GetWindowSize());
    ImGui::SetNextWindowPos({0,0});

    bool windowStarted = ImGui::Begin("ImGraph", nullptr,
      ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

    if (!windowStarted) {
      ImGui::End();
      return;
    }

    // Ask for configuration file.
    ImGui::OpenPopup("Input Configuration File");
    if (!ImGui::BeginPopupModal("Input Configuration File"))
      return;

    ImGui::Text("Provide a path to the input configuration file...");

    NFD_Init();

    nfdu8char_t *outPath;
    nfdu8filteritem_t filters[2] = { { "Configuration file", "yaml,YAML,yml" }};
    nfdopendialogu8args_t args = {0};
    args.filterList = filters;
    args.filterCount = 1;
    nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
    if (result == NFD_OKAY)
    {
        std::cout << "Success!" << '\n';
        std::cout << outPath << '\n';
        NFD_FreePathU8(outPath);
    }
    else if (result == NFD_CANCEL)
    {
        std::cout << "User pressed cancel." << '\n';
    }
    else
    {
        std::cout << "Error: " << NFD_GetError() << '\n';
    }

    ImGui::EndPopup();

    ImGui::End();
  }

};

int main(int argc, char const *argv[])
{
  InterfaceWindow testInterface {"Line Follower Tracker", 640, 480, argc, argv};
  testInterface.Run();

  return 0;
}
