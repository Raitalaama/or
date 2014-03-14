/*
 * io_handler.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: risto
 */

#include "or_common/io_handler.h"

using boost::filesystem::path;
using boost::filesystem::directory_iterator;
using std::string;
using std::stringstream;


void IOHandler::setInputDirectory(string input_directory)
{

  path tmp(input_directory);
  if (exists(tmp) && is_directory(tmp))
  {
    input_directory_ = tmp;


  }
  else
    printf("Not a valid input directory, old one kept / weird behaviour incoming");

// TODO Make better else result
}

void IOHandler::setOutputDirectory(std::string output_directory)
{
  path tmp(output_directory);
  if (exists(tmp) && is_directory(tmp))
  {
    output_directory_ = tmp;
  }
  else
    printf("Not a valid output directory, old directory kept / weird behavior incoming");

  //TODO when its not a valid directory, try to create it
}


void IOHandler::loadInputFiles()
{
  files_.clear();

      directory_iterator current = directory_iterator(input_directory_);

      while (current != directory_iterator())
      {
        string ext = extension((*current).path());
        string ext_lower;
        ext_lower.reserve(ext.size());
        transform(ext.begin(), ext.end(), back_inserter(ext_lower), static_cast<int (*)(int)>(std::tolower));if
  (      ext_lower==("."+filename_extension_))
        {
          files_.push_back((*current).path());
          //printf("file with name %s added to the file list", ((*current).path().filename()));
        }
        ++current;
      }
      sort(files_.begin(), files_.end());
      files_iterator_ = files_.begin();
}

std::string IOHandler::getCompleteInputFilename()
{
  stringstream input_filename;
  input_filename<<input_directory_.string()<<"/"<<filename_body_<<"."<<filename_extension_;
  return input_filename.str();
}

 std::string IOHandler::getCurrentIteratedFilename() const
{
  //path current = *files_iterator_;
  return (*files_iterator_).filename().string();
}

std::string IOHandler::getCompleteOutputFilename(bool always_use_index)
{
  //std::cout<<"getCompleteOutputFilename:\n"<<"output dir: "<<output_directory_.string()<<"/n";
  stringstream output_filename;
  bool use_index = always_use_index;
  stringstream temp_filename;
  output_filename << output_directory_.string() << "/" << filename_body_;
  //std::cout<<"With / and body: "<<output_filename.str()<<"\n";
  int i = 1;
  do
  {
    temp_filename.str(std::string());
    temp_filename << output_filename.str();
    if (use_index)
      temp_filename << i;
    temp_filename << "." << filename_extension_;
    //std::cout<<"With . and extension: "<<temp_filename.str()<<"\n";

    if (!use_index && exists(path(temp_filename.str())))
    {
      ROS_INFO("File with current filename exists, adding index");
      use_index = true;
    }
    else
      ++i;
  } while (exists(path(temp_filename.str())));
  return temp_filename.str();
}
void IOHandler::setFilenameExtension(std::string filenameExtension)
{
  filename_extension_ = filenameExtension;
}

IOHandler::IOHandler()
{
  // TODO Auto-generated constructor stub

}

IOHandler::~IOHandler()
{
  // TODO Auto-generated destructor stub
}

