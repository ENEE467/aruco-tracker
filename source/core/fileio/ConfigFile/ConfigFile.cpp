#include <filesystem>

#include "ConfigFile.hpp"

namespace fileio {

ConfigFile::ConfigFile(
  const std::string& configDirectoryIn,
  const std::string& nameIn,
  bool isTemplateFileIn)
: File {configDirectoryIn, "config", nameIn, "yaml"},
  _isTemplateFile {isTemplateFileIn}
{}

ConfigFile::ConfigFile(const std::string& filePathIn)
: File {filePathIn},
  _cvFileObject {_filePath, cv::FileStorage::READ}
{
  if (!_cvFileObject.isOpened())
    throw std::runtime_error("Config file cannot be opened for reading.");

  _trackingOptions.readFromConfigFile(_cvFileObject);
  _calibrationOptions.readFromConfigFile(_cvFileObject);

  _cvFileObject.release();
}

void ConfigFile::saveFile()
{
  if (std::filesystem::is_regular_file(_filePath))
    std::filesystem::remove(_filePath);

  _cvFileObject.open(_filePath, cv::FileStorage::WRITE);

  if (!_cvFileObject.isOpened())
    throw std::runtime_error("Config file cannot be opened for writing.");

  _trackingOptions.detection.writeToConfigFile(_cvFileObject);
  _trackingOptions.lineFollowerMarker.writeToConfigFile(_cvFileObject);
  _trackingOptions.boardMarkers.writeToConfigFile(_cvFileObject);
  _trackingOptions.track->writeToConfigFile(_cvFileObject);
  _calibrationOptions.calibrationBoard.writeToConfigFile(_cvFileObject);
  _calibrationOptions.intrinsicParams.writeToConfigFile(_cvFileObject);

  if (!_isTemplateFile) {
    _cvFileObject.release();
    return;
  }

  // Also add default fields for the remaining tracks if it's a template config file.
  for (const auto& trackType: tracks::Types) {
    if (trackType.first == _trackingOptions.track->getType())
      continue;

    if (trackType.first == tracks::Type::LINE)
      tracks::LineTrack().writeToConfigFile(_cvFileObject);

    else if (trackType.first == tracks::Type::ROUND)
      tracks::RoundTrack().writeToConfigFile(_cvFileObject);

    else if (trackType.first == tracks::Type::POLYGON)
      tracks::PolygonTrack().writeToConfigFile(_cvFileObject);
  }

  _cvFileObject.release();
}

}
