#include <gtest/gtest.h>

#include "fileio.hpp"

// createPath() method tests -----------------------------------------------------------------------

/**
 * Test 1:
 *  Basic output test to generate a directory path with a timestamped name
 *
 *  Output path without backslash at the end of parent directory
 *  Prefix is given
 *  Name isn't given to generate timestamped name
 *  No extension to test the validity of generated directory
 */
TEST(FileIOTest, createPathTest1)
{
  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  auto expectedTimeStamp {std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")};

  auto outputPath {fileio::createPath("test_output", "test_dir", "", "").str()};

  std::stringstream expectedPath;
  expectedPath << "test_output/" << "test_dir-" << expectedTimeStamp;

  EXPECT_STREQ(expectedPath.str().c_str(), outputPath.c_str());
}

/**
 * Test 2:
 *  Tests if the method could handle the backslash entry at the end of parent directory
 *
 *  Rest of the parameters are same as the Test 1
 */
TEST(FileIOTest, createPathTest2)
{
  auto outputPathNoBackSlash {fileio::createPath("test_output", "test_dir", "", "").str()};
  auto outputPathBackSlash {fileio::createPath("test_output/", "test_dir", "", "").str()};

  EXPECT_STREQ(outputPathBackSlash.c_str(), outputPathNoBackSlash.c_str());
}

/**
 * Test 3:
 *  Basic output test to generate a file path with extension and timestamped name
 *
 *  Output path with backslash at the end of parent directory
 *  Prefix is given
 *  Name isn't given to generate timestamped name
 *  Extension is given
 */
TEST(FileIOTest, createPathTest3)
{
  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  auto expectedTimeStamp {std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")};

  auto outputPath {fileio::createPath("test_output/", "test_file", "", "ext").str()};

  std::stringstream expectedOutputPath;
  expectedOutputPath << "test_output/" << "test_file-" << expectedTimeStamp << ".ext";

  EXPECT_STREQ(expectedOutputPath.str().c_str(), outputPath.c_str());
}

/**
 * Test 4:
 *  Basic output test to generate a directory with custom name
 *
 *  Output path without backslash at the end of parent directory
 *  Prefix is given
 *  Name is given
 *  Extension isn't given since it's a directory
 */
TEST(FileIOTest, createPathTest4)
{
  auto outputPath {fileio::createPath("test_output", "test_dir", "test_name", "").str()};

  EXPECT_STREQ("test_output/test_dir-test_name", outputPath.c_str());
}

/**
 * Test 5:
 *  Basic output test to generate a file path with custom name (and extension of course!)
 *
 *  Output path with backslash at the end of parent directory
 *  Prefix is given
 *  Name is given
 *  Extension is given
 */
TEST(FileIOTest, createPathTest5)
{
  auto outputPath {fileio::createPath("test_output/", "test_file", "test_name", "ext").str()};

  EXPECT_STREQ("test_output/test_file-test_name.ext", outputPath.c_str());
}

/**
 * Test 6:
 *  Nested file creation by recursive calling with timestamped name.
 *
 *  Output path with backslash at the end of parent directory
 *  Prefix is given
 *  Name is not given to create a timestamped name
 *  Extension is given
 */
TEST(FileIOTest, createPathTest6)
{
  std::time_t t {std::time(nullptr)};
  std::tm tm {*std::localtime(&t)};
  auto expectedTimeStamp {std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")};

  auto outputDir {fileio::createPath("test_output/", "test_dir", "", "").str()};
  auto outputFile {fileio::createPath(outputDir, "test_file", "", "ext").str()};

  std::stringstream expectedOutputPath;
  expectedOutputPath << "test_output/" << "test_dir-" << expectedTimeStamp << "/"
                     << "test_file-" << expectedTimeStamp << ".ext";

  EXPECT_STREQ(expectedOutputPath.str().c_str(), outputFile.c_str());
}

/**
 * Test 7:
 *  Nested file creation by recursive calling with custom name.
 *
 *  Output path without backslash at the end of parent directory
 *  Prefix is given
 *  Name is given
 *  Extension is given
 */
TEST(FileIOTest, createPathTest7)
{
  auto outputDir {fileio::createPath("test_output", "test_dir", "test_name", "").str()};
  auto outputFile {fileio::createPath(outputDir, "test_file", "test_name1", "ext").str()};

  EXPECT_STREQ("test_output/test_dir-test_name/test_file-test_name1.ext", outputFile.c_str());
}
