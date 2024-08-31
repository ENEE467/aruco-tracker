#include "CSVFile.hpp"

namespace fileio {

CSVFile::CSVFile(
  const std::string& outputDirectoryIn,
  const std::string& prefixIn,
  const std::string& outputNameIn
)
: File {outputDirectoryIn, prefixIn, outputNameIn, "csv"},
  _outputCSVFile {_filePath}
{
  if (!_outputCSVFile.is_open())
    throw std::runtime_error("Cannot open the CSV file");
}

void CSVFile::writePosition(const cv::Point2d& positionIn, const double timeSecondsIn)
{
  _outputCSVFile << positionIn.x << ", " << positionIn.y << ", " << timeSecondsIn << '\n';
}

void CSVFile::writeError(const double errorIn, const double timeSecondsIn)
{
  _outputCSVFile << errorIn << ", " << timeSecondsIn << '\n';
}

}
