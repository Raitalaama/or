/*
 * io_handler.h
 *
 *  Created on: Jul 9, 2012
 *      Author: risto
 */

#ifndef IOHANDLER_H_
#define IOHANDLER_H_

#include "or_common/typedefs.h"

#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>



class IOHandler
{
public:
  IOHandler();
  virtual ~IOHandler();

  void setInputDirectory(std::string input_directory);
  void setOutputDirectory(std::string output_directory);

  std::string getInputDirectory() const
  {
    return input_directory_.string();
  }

  std::string getOutputDirectory() const
  {
    return output_directory_.string();
  }

  std::string getFilenameBody() const
  {
    return filename_body_;
  }

  void setFilenameBody(std::string filenameBody)
  {
    filename_body_ = filenameBody;
  }

  std::string getFilenameExtension() const
  {
    return filename_extension_;
  }

  void setFilenameExtension(std::string filenameExtension);
  void loadInputFiles();
  std::string getCurrentIteratedFilename() const;
  std::string getCompleteOutputFilename(bool always_use_index = true);
  std::string getCompleteInputFilename();


  std::vector<boost::filesystem::path> files_;

protected:


  boost::filesystem::path input_directory_;
  boost::filesystem::path output_directory_;
  std::string filename_body_;
  std::string filename_extension_;
  std::vector<boost::filesystem::path>::iterator files_iterator_;
};

#endif /* IOHANDLER_H_ */
