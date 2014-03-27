/*
 * segmenter.h
 *
 *  Created on: Mar 6, 2014
 *      Author: raitalaama
 */

#include "or_common/helper_functions.h"
#include "or_common/file_handler.h"
#include "or_common/meta_data.h"

#include "or_common/typedefs.h"
#include <vector>
#include <algorithm>
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
  Segmenter(std::string node_namespace);
  virtual ~Segmenter();


  void segment();

  void setInputCloud(Cloud::Ptr input);
  void setInputIndices(pcl::PointIndices::Ptr input);


  void setInputClass(std::string input_class);
  void setInputPipeline(std::string input_pipeline);

  std::vector<or_common::MetaData> getResultMetaData();
  std::vector<pcl::PointIndices::Ptr> getResultIndices();



private:
  void loadSettings();

  void initialPassThrough();
  void finalPassThrough();
  void crCbSkinFiltering();
  void euclideanClustering();
  void tableTopFiltering();
  bool isSkin(const cv::Mat skin_model,const cv::Vec3b color);
  pcl::PointIndices::Ptr filterSkin(const cv::Mat skin_model, pcl::PointIndices::Ptr indices); //TODO ref?

  void saveImage(Cloud::Ptr cloud, or_common::MetaData meta_info);
  void saveImage(Cloud::Ptr cloud,pcl::PointIndices::Ptr indices, or_common::MetaData meta_info);
  void saveImage(Cloud::Ptr cloud,pcl::PointIndices::Ptr indices);
  void saveImage(Cloud::Ptr cloud);
  void saveImage(Cloud::Ptr cloud, std::vector<int> indices);




  Cloud::Ptr input_cloud_;
  std::vector<pcl::PointIndices::Ptr> indices_;

  std::string input_class_; //TODO really needed as member variable?
  std::string input_pipeline_; //TODO use or remove


  std::vector<or_common::MetaData> meta_;

  ros::NodeHandle nh_;
  FileHandler fh_;
  bool settings_loaded_;

  bool perform_initial_pass_through_filtering_;
  bool perform_final_pass_through_filtering_;
  bool perform_crcb_skin_filtering_;
  bool perform_euclidean_clustering_;
  bool perform_table_top_filtering_;

  std::vector<or_common::MetaData> crcb_skin_parameters_;
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

