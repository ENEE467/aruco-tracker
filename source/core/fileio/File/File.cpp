#include <ctime>
#include <iomanip>
#include <filesystem>
#include <cassert>

#include "File.hpp"

namespace fileio {

std::string createTimeStamp()
{
  std::stringstream timeStamp;

  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  timeStamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  return timeStamp.str();
}

std::string createPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& nameIn,
  const std::string& extensionIn)
{
  assert(!parentDirectoryIn.empty());

  std::string outputName {nameIn};
  if (outputName.empty())
    outputName = createTimeStamp();

  std::string parentDirectory {parentDirectoryIn};
  if (parentDirectory.back() != '/')
    parentDirectory.push_back('/');

  std::string prefix {prefixIn};
  if (!prefix.empty() && prefix.back() != '-')
    prefix.push_back('-');

  bool isPathToDirectory {true};
  std::string fileExtension {};
  if (!extensionIn.empty()) {
    isPathToDirectory = false;
    fileExtension = extensionIn.front() == '.' ? "" : '.' + extensionIn;
  }

  auto pathAlreadyExists = [&isPathToDirectory](const std::stringstream& outputPathIn)
  {
    return ( isPathToDirectory && std::filesystem::is_directory(outputPathIn.str()) )
      || ( !isPathToDirectory && std::filesystem::is_regular_file(outputPathIn.str()) );
  };

  int duplicateCounter {1};
  std::stringstream outputPathStream;
  outputPathStream << parentDirectory << prefix << nameIn << fileExtension;

  while (pathAlreadyExists(outputPathStream)) {
    outputPathStream.str(std::string());  // clears/resets the stream object to write new path
    outputPathStream << parentDirectory << prefix << nameIn << duplicateCounter << fileExtension;

    duplicateCounter++;
  }

  return outputPathStream.str();
}

File::File(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& nameIn,
  const std::string& extensionIn)
{
  if (parentDirectoryIn.empty())
    throw std::runtime_error("Path to parent directory is not given");

  if(extensionIn.empty())
    throw std::runtime_error("File must have an extension");

  _fileName = nameIn.empty() ? createTimeStamp() : nameIn;
  _filePath = createPath(parentDirectoryIn, prefixIn, _fileName, extensionIn);
}

File::File(const std::string& pathIn)
{
  if (pathIn.empty())
    throw std::runtime_error("Given path cannot be empty when initializing a file");

  _filePath = pathIn;
}

}
