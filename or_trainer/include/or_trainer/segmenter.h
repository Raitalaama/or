/*
 * segmenter.h
 *
 *  Created on: Mar 6, 2014
 *      Author: raitalaama
 */

#include "or_common/helper_functions.h"
#include "or_common/file_handler.h"

#include "or_common/typedefs.h"
#include <vector>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <ros/ros.h>





#ifndef SEGMENTER_H_
#define SEGMENTER_H_

namespace or_training
{

class Segmenter
{
public:
  Segmenter();
  virtual ~Segmenter();


  void segment();

  void setInputCloud(Cloud::Ptr input);

  void setInputClass(std::string input_class);
  void setInputPipeline(std::string input_pipeline);

  std::vector<std::string> getResultMetaData();
  std::vector<Cloud::Ptr> getResultPointCloud();


private:
  void loadSettings();

  void initialPassThrough();
  void finalPassThrough();
  void crCbSkinFiltering();
  void euclideanClustering();
  void tableTopFiltering();
  bool isSkin(const cv::Mat skin_model,const cv::Vec3b color);
  void filterSkin(const cv::Mat skin_model, Cloud::Ptr &cloud); //TODO ref?

  void saveImage(Cloud::Ptr cloud, std::string meta_info);


  Cloud::Ptr input_;
  std::vector<Cloud::Ptr> output_;

  std::string input_class_;
  std::string input_pipeline_; //TODO use or remove


  std::vector<std::string> meta_output_;

  ros::NodeHandle nh_;
  FileHandler fh_;
  bool settings_loaded_;

  bool perform_initial_pass_through_filtering_;
  bool perform_final_pass_through_filtering_;
  bool perform_crcb_skin_filtering_;
  bool perform_euclidean_clustering_;
  bool perform_table_top_filtering_;

  std::vector<std::string> crcb_skin_parameters_;
  std::vector<cv::Mat> crcb_skin_model_;
  std::vector<int> erode_dilate_mask_size_;

  pcl::PointIndices::Ptr skin_indices_;

  std::vector<pcl::EuclideanClusterExtraction<PointRGB> > euclidean_clustering_;

  std::vector<pcl::PassThrough<PointRGB> > initial_pass_through_x_;
  std::vector<pcl::PassThrough<PointRGB> > initial_pass_through_y_;
  std::vector<pcl::PassThrough<PointRGB> > initial_pass_through_z_;
  std::vector<pcl::PassThrough<PointRGB> > final_pass_through_x_;
  std::vector<pcl::PassThrough<PointRGB> > final_pass_through_y_;
  std::vector<pcl::PassThrough<PointRGB> > final_pass_through_z_;

  std::vector<float> table_plane_x_;
  std::vector<float> table_plane_y_;
  std::vector<float> table_plane_z_;

};

} /* namespace or_training */

#endif /* SEGMENTER_H_ */

