/*
 * ORTrainer.h
 *
 *  Created on: Mar 6, 2014
 *      Author: raitalaama
 */

#ifndef ORTRAINER_H_
#define ORTRAINER_H_

#include <ros/ros.h>
#include "or_common/typedefs.h"
#include "or_common/bag_handler.h"
#include "or_common/feature_extract.h"

#include "or_trainer/segmenter.h"


namespace or_training
{

class ORTrainer
{
public:
  ORTrainer();
  virtual ~ORTrainer();
  void processPointCloud();
  void train();
  void loadSettings();

private:
  bool live_input_;
  bool settings_loaded_;
  int process_every_th_frame_;
  std::string bag_input_dir_;
  std::string png_output_dir_;
  ros::NodeHandle nh_;
  BagHandler bag_handler_;



  std::vector<pcl::PointIndices::Ptr> point_indices_;

  Cloud::Ptr input_cloud_;

};

} /* namespace or_training */

#endif /* ORTRAINER_H_ */
