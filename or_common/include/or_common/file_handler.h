/*
 * file_handler.h
 *
 *  Created on: Jun 7, 2012
 *      Author: risto
 */

#ifndef FILEHANDLER_H_
#define FILEHANDLER_H_

#include "or_common/typedefs.h"
#include "or_common/io_handler.h"
#include "or_common/helper_functions.h"

#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <boost/filesystem.hpp>

#include <boost/filesystem/operations.hpp>

#include <stdio.h>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

class FileHandler : public IOHandler
{
public:
  FileHandler();
  virtual ~FileHandler();

  void loadAnnotatedFeaturesDir(std::vector<cv::Mat>* features, std::vector<cv::Mat>* classes);

  void saveImage(cv::Mat image);
  void saveImage(cv::Mat image, std::string filename);

  void saveFeatures(const cv::Mat data);
  void saveTrainingData(const cv::Mat features, const cv::Mat classes);
  void saveTrainingData(const std::vector<cv::Mat> features, const std::vector<cv::Mat> classes);

  void loadTrainingDataDir(std::vector<cv::Mat>* features, std::vector<cv::Mat>* classes,
                           float percentage_of_features_used);
  void loadTrainingData(cv::Mat features, cv::Mat classes);

  void savePointCloud(Cloud::ConstPtr cloud);
  void savePointCloud(Cloud::ConstPtr cloud, const pcl::PointIndices::ConstPtr &indices);
  void savePointCloud(Cloud::ConstPtr cloud, std::vector<pcl::PointIndices> &indice_vector);

  void setLoopInput(bool loop_files);
  Cloud::Ptr getNextPointCloud();

  std::vector<int> getNonUsedClasses() const
  {
    return non_used_classes_;
  }

  void setNonUsedClasses(std::vector<int> nonUsedClasses)
  {
    non_used_classes_ = nonUsedClasses;
  }

protected:
  bool loop_input_files;
  std::vector<int> compression_params;
  std::vector<int> non_used_classes_;
  bool isAllowedClass(int class_name);

};

#endif /* FILEHANDLER_H_ */
