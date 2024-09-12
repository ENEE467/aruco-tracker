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

  if (_trackingOptions.track->getType() != tracks::Type::NONE)
    _trackingOptions.track->writeToConfigFile(_cvFileObject);

  _calibrationOptions.calibrationBoard.writeToConfigFile(_cvFileObject);
  _calibrationOptions.intrinsicParams.writeToConfigFile(_cvFileObject);

  if (!_isTemplateFile) {
    _cvFileObject.release();
    return;
  }

  // Also add default fields for the remaining tracks if it's a template config file.
  _cvFileObject.writeComment(
    "\nTemplate parameters for defining tracks. Populate track parameters for ONLY ONE of these"
    "\ntypes and omit the rest. If multiple tracks were left defined in the config file, only the"
    "\nfirst valid track configuration that's found is read and the rest are ignored.");

  for (const auto& trackType: tracks::Types) {
    if (trackType.first == _trackingOptions.track->getType())
      continue;

    switch (trackType.first) {

    case tracks::Type::LINE:
      tracks::LineTrack().writeToConfigFile(_cvFileObject);
      break;

    case tracks::Type::ARC:
      tracks::ArcTrack().writeToConfigFile(_cvFileObject);
      break;

    case tracks::Type::ROUND:
      tracks::RoundTrack().writeToConfigFile(_cvFileObject);
      break;

    case tracks::Type::POLYGON:
      tracks::PolygonTrack().writeToConfigFile(_cvFileObject);
      break;

    case tracks::Type::PILL:
      tracks::PillTrack().writeToConfigFile(_cvFileObject);
      break;

    default:
      break;

  }
}
}

}
