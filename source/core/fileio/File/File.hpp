#pragma once

#include <string>

namespace fileio {

std::string createTimeStamp();
std::string createPath(
  const std::string& parentDirectoryIn,
  const std::string& prefixIn,
  const std::string& nameIn,
  const std::string& extensionIn);

class File {

public:
  virtual bool isOpen() const = 0;

  const std::string& getName() const {return _fileName;}
  const std::string& getPath() const {return _filePath;}

protected:
  File(
    const std::string& parentDirectoryIn,
    const std::string& prefixIn,
    const std::string& nameIn,
    const std::string& extensionIn);

  File(const std::string& pathIn);

  std::string _fileName {};
  std::string _filePath {};

};

}
