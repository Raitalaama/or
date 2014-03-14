/*
 * ORTrainer.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: raitalaama
 */

#include "or_trainer/or_trainer.h"

namespace or_training
{

ORTrainer::ORTrainer() : nh_("or_trainer/")
{
  // TODO Auto-generated constructor stub
  loadSettings();
}

ORTrainer::~ORTrainer()
{
  // TODO Auto-generated destructor stub
}

void ORTrainer::train()
{
  if (!settings_loaded_)
  {
    ROS_ERROR("Could not load all required or_trainer settings, terminating");
    return;
  }
  ros::Rate rate(5);

  bag_handler_.setInputDirectory(bag_input_dir_);
  bag_handler_.setEveryThFrame(process_every_th_frame_);
  bag_handler_.loadBagsFromDir();

  while (ros::ok() && bag_handler_.getMsgsLeft())
  {
    //ros::spinOnce();
    input_cloud_ = bag_handler_.getNextPointCloud();

    processPointCloud();
    rate.sleep();
  }

}

void ORTrainer::processPointCloud()
{
  ROS_INFO("prosessointi alko!!");

  Segmenter segmenting;
  segmenting.setInputCloud(input_cloud_);
  segmenting.setInputClass(bag_handler_.getCorrespondingClassName());
  //segmenting.setInputPipeline(bag_handler_.getCorrespondingPipeline());
  segmenting.setInputPipeline("hand"); //TODO use previous line
  segmenting.segment();
  output_clouds_ = segmenting.getResultPointCloud();

  ROS_INFO("prosessointi onnas!!");
}

void ORTrainer::loadSettings()
{
//TODO make throw exception with info about what setting
  settings_loaded_ = true;
  if (!nh_.getParam("live_input", live_input_))
    settings_loaded_ = false;
  if (!nh_.getParam("bag_handler/process_every_th_frame", process_every_th_frame_))
    settings_loaded_ = false;
  if (!nh_.getParam("bag_handler/bag_input_dir", bag_input_dir_))
    settings_loaded_ = false;
  if (!nh_.getParam("png_output_dir", png_output_dir_))
    settings_loaded_ = false;
}

} /* namespace or_training */

