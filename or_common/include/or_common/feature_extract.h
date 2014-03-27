/*
 * FeatureExtract.h
 *
 *  Created on: Mar 15, 2014
 *      Author: raitalaama
 */

#ifndef FEATUREEXTRACT_H_
#define FEATUREEXTRACT_H_

#include "or_common/typedefs.h"
#include "or_common/helper_functions.h"

#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/uniform_sampling.h>


namespace or_common
{

class FeatureExtract
{
public:
  FeatureExtract();
  FeatureExtract(std::string node_namespace);

  virtual ~FeatureExtract();

  std::vector<CloudN::Ptr> getNormals();
  void setInputCloud(Cloud::Ptr input_cloud);

  void setInputMeta(std::string input_meta);

  void setInputNormals(std::vector<CloudN::Ptr>  input_normals);


  void setInputIndices(pcl::PointIndices::Ptr input_indices);

  void computeFPFHDescriptors();
  void computeUniformKeypoints();
  std::vector<cv::Mat> getFPFHDescriptorsCV();
  std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr > getFPFHDescriptors();

private:

  void computeNormals();
  ros::NodeHandle nh_;
  void loadSettings();

  std::string input_meta_;

  Cloud::Ptr input_cloud_;
  std::vector<CloudN::Ptr>  normals_;
  pcl::PointIndices::Ptr input_indices_;

  bool have_input_indices_;
  bool have_normals_;

  std::vector<std::string> meta_output_;

  std::vector<Cloud::Ptr> keypoints_;
  std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr > fpfh_descriptors_;

  std::vector<cv::Mat> fpfh_descriptors_cv_;
  std::vector<pcl::IntegralImageNormalEstimation<PointRGB, PointN> > normal_estimators_;
  std::vector<pcl::UniformSampling<PointRGB> > uniform_sampling_;
  std::vector<pcl::FPFHEstimationOMP<PointRGB, PointN, pcl::FPFHSignature33> > fpfh_estimators_;

};

} /* namespace or_common */

#endif /* FEATUREEXTRACT_H_ */
