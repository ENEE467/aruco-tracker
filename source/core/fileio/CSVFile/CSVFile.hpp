#pragma once

#include <string>
#include <fstream>

#include <opencv2/core/types.hpp>

#include "File/File.hpp"

namespace fileio {

class CSVFile : public File {

public:
  CSVFile(
    const std::string& outputDirectoryIn,
    const std::string& prefixIn = "",
    const std::string& outputName = "");

  bool isOpen() const override {return _outputCSVFile.is_open();}

  void writePosition(const cv::Point2d& positionIn, const double timeSecondsIn = 0);
  void writeError(const double errorIn, const double timeSecondsIn);

private:
  std::ofstream _outputCSVFile;

};

}
