/*
 * helper_functions.h
 *
 *  Created on: May 31, 2012
 *      Author: risto
 */

#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include "or_common/typedefs.h"

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

#include <vector>
#include <string>
#include <math.h>

#include <opencv2/opencv.hpp>

#include <boost/math/special_functions/fpclassify.hpp>



namespace helper_functions
{


//Cloud::Ptr filterIndices(const Cloud::ConstPtr &input_cloud, const pcl::PointIndices::ConstPtr &indices, bool negative=false);

pcl::PointIndices::Ptr combineIndices(std::vector<pcl::PointIndices> &indice_vector);
cv::Mat pcTocv(Cloud::Ptr &input_cloud, std::string window_name);
cv::Mat pcTocv(Cloud::Ptr &input_cloud);
cv::Mat pcTocv(Cloud::Ptr &input_cloud, pcl::PointIndices::Ptr indices);
cv::Mat pcTocv(Cloud::Ptr &input_cloud, IndicesPtr indices);
cv::Mat pcTocv(Cloud::Ptr &input_cloud, std::vector<int> indices);




cv::Mat insertText(cv::Mat image,std::string line1="",std::string line2="",std::string line3="",std::string line4="",std::string line5="",std::string line6="");

pcl::PointIndices::Ptr getFiniteIndices(Cloud::ConstPtr input_cloud);


cv::Mat pcToDistcv(Cloud::Ptr &input_cloud);


cv::Mat highlightPoints(cv::Mat base_image, boost::shared_ptr<std::vector<int> > points);

std::vector<int> getIntSettings(std::string setting_name);
std::vector<float> getFloatSettings(std::string setting_name);

std::string getStringSetting(std::string setting_name);
std::vector<std::string> getStringVectorSetting(std::string setting_name);


}

#endif /* HELPERFUNCTIONS_H_ */
