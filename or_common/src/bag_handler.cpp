/*
 * bag_handler.cpp
 *
 *  Created on: Jun 17, 2012
 *      Author: risto
 */

#include "or_common/bag_handler.h"

//TODO inherit filehandler

BagHandler::BagHandler()
{
  every_th_frame = 1;
  msgs_left = false;
  pc_recieved = false;
  iterate_dir_ = true;
  d_img = "camera/depth_registered/image_raw";
  d_info = "camera/depth_registered/camera_info";
  rgb_img = "camera/rgb/image_raw";
  rgb_info = "camera/rgb/camera_info";

  pc_subscriber = nh_.subscribe("camera/depth_registered/points", 1, &BagHandler::point_cloud_cb, this);

  depth_image_pub = nh_.advertise<sensor_msgs::Image>("camera/depth_registered/image_raw", 1);
  rgb_image_pub = nh_.advertise<sensor_msgs::Image>("camera/rgb/image_raw", 1);
  rgb_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 1);
  depth_info_pub = nh_.advertise<sensor_msgs::CameraInfo>("camera/depth_registered/camera_info", 1);

  filename_extension_ = "bag";
  ros::Duration(1).sleep();

}

BagHandler::~BagHandler()
{
  // TODO Auto-generated destructor stub
}

void BagHandler::loadBag(const std::string& filename)
{
  view_ptr_.reset(new rosbag::View);
  iterate_dir_ = false;
  bag.close();
  bag.open(filename, rosbag::bagmode::Read);
  view_ptr_->addQuery(bag);
  view_it_ = view_ptr_->begin();
  view_it_end_ = view_ptr_->end();
  if (view_it_ != view_it_end_)
    msgs_left = true;
  message_index = 0;
//  std::cout << "bagname: "<<bag.getFileName()<<"bagsize: "<<bag.getSize()<<"\n";
}

void BagHandler::setEveryThFrame(int every_th)
{
  every_th_frame = every_th;
}

bool BagHandler::getMsgsLeft()
{
  return msgs_left;
}

Cloud::Ptr BagHandler::getNextPointCloud()
{
  Cloud::Ptr tmp;
  tmp.reset(new Cloud());
  tmp = cloud;
  if (!msgs_left)
  {
    return tmp;
  }
  pc_recieved = false;

  while (!pc_recieved)
  {
//    ROS_INFO("baghander: pc not received");

    if ((*view_it_).getTopic() == d_img || ("/" + (*view_it_).getTopic() == d_img))
    {
      sensor_msgs::Image::ConstPtr d_image = (*view_it_).instantiate<sensor_msgs::Image>();
      if (d_image != NULL)
        depth_image_pub.publish(d_image);
    }

    if ((*view_it_).getTopic() == rgb_img || ("/" + (*view_it_).getTopic() == rgb_img))
    {
      sensor_msgs::Image::ConstPtr rgb_image = (*view_it_).instantiate<sensor_msgs::Image>();
      if (rgb_image != NULL)
        rgb_image_pub.publish(rgb_image);
    }

    if ((*view_it_).getTopic() == d_info || ("/" + (*view_it_).getTopic() == d_info))
    {
      sensor_msgs::CameraInfo::ConstPtr d_camera_info = (*view_it_).instantiate<sensor_msgs::CameraInfo>();
      if (d_camera_info != NULL)
        depth_info_pub.publish(d_camera_info);
    }

    if ((*view_it_).getTopic() == rgb_info || ("/" + (*view_it_).getTopic() == rgb_info))
    {
      sensor_msgs::CameraInfo::ConstPtr rgb_camera_info = (*view_it_).instantiate<sensor_msgs::CameraInfo>();
      if (rgb_camera_info != NULL)
        rgb_info_pub.publish(rgb_camera_info);
    }

    ++view_it_;
    ++message_index;
    if (view_it_ == view_it_end_)
      break;

    ros::spinOnce();
    ros::Duration(0.12).sleep();
  }
  tmp = cloud;
  while ((message_index % (4 * every_th_frame)) != 0 && view_it_ != view_it_end_)
  {
    ++view_it_;
    ++message_index;
  }
  if (view_it_ == view_it_end_)
  {
    if (iterate_dir_)
    {
      loadNextBagFromDir();
    }
    else
    {
      msgs_left = false;
    }
  }
  return tmp;

}

void BagHandler::loadNextBagFromDir()
{
  view_ptr_.reset(new rosbag::View);
  bag.close();
  if (files_iterator_ != files_.end())
  {
    bag.open((*files_iterator_).string(), rosbag::bagmode::Read);
    std::cout<<"Opening file: "<<(*files_iterator_).string()<<"\n";

    /*if((*files_iterator_).string()=="/home/risto/ros_workspace/recording/objects_in_hand3/cup_5.bag")
    view_ptr_->addQuery(bag,ros::Time(1341929090.98),ros::TIME_MAX);
    else*/
      view_ptr_->addQuery(bag);

    view_it_ = view_ptr_->begin();
    view_it_end_ = view_ptr_->end();
    if (view_it_ != view_it_end_)
      msgs_left = true;
    message_index = 0;
    ++files_iterator_;
  }
  else
  {
    msgs_left = false;
  }
  ros::Duration(1.5).sleep();

}

void BagHandler::loadBagsFromDir()
{
  iterate_dir_ = true;
  loadInputFiles();
  loadNextBagFromDir();
}

std::string BagHandler::getCorrespondingClassName() const
{
  using boost::filesystem::path;
  path meta_file = change_extension(path(bag.getFileName()),path(".yaml"));
  std::string class_name = "no_class_defined";
  if(!exists(meta_file))
  {
    ROS_INFO("%s",meta_file.c_str());
    ROS_INFO("No metafile corresponding to bag found, using class no_class_defined");
  }
  else
  {
    //std::cout<<"metafile: "<<meta_file.string()<<"\n";
    cv::FileStorage fs(meta_file.string(), cv::FileStorage::READ);
     fs["Class"]>>class_name;
     fs.release();
  }
  return class_name;
}

std::string BagHandler::getCorrespondingPipeline() const
{
  using boost::filesystem::path;
  path meta_file = change_extension(path(bag.getFileName()),path(".yaml"));
  std::string pipeline_name = "hand";
  if(!exists(meta_file))
  {
    ROS_INFO("%s",meta_file.c_str());
    ROS_INFO("No metafile corresponding to bag found, using hand pipeline");
  }
  else
  {
    //std::cout<<"metafile: "<<meta_file.string()<<"\n";
    cv::FileStorage fs(meta_file.string(), cv::FileStorage::READ);
     fs["Pipeline"]>>pipeline_name;
     fs.release();
  }
  return pipeline_name;
}

void BagHandler::point_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
 // ROS_INFO("bagPC callback called");
  if (!pc_recieved)
  {
    mutex.lock();
    cloud.reset(new Cloud);
    pcl::fromROSMsg(*msg, *cloud);
    mutex.unlock();
    pc_recieved = true;
  }
}
