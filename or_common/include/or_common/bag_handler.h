/*
 * bag_handler.h
 *
 *  Created on: Jun 17, 2012
 *      Author: risto
 */

#ifndef BAGHANDLER_H_
#define BAGHANDLER_H_

#include "or_common/typedefs.h"
#include "or_common/io_handler.h"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <boost/thread/mutex.hpp>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class BagHandler : public IOHandler
{
public:

  BagHandler();
  virtual ~BagHandler();

  void loadBagsFromDir();

  void loadBag(const std::string &filename);


  bool getMsgsLeft();

  void setEveryThFrame(int every_th);
  Cloud::Ptr getNextPointCloud();

  std::string getCorrespondingClassName() const;
  std::string getCorrespondingPipeline() const;



private:
  ros::NodeHandle nh_;
  void loadNextBagFromDir();


  std::string d_img, rgb_img, d_info, rgb_info;
  ros::Publisher depth_image_pub, depth_info_pub, rgb_image_pub, rgb_info_pub;

  void point_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg);

  sensor_msgs::Image depth_image_tmp;
  sensor_msgs::Image rgb_image_tmp;
  sensor_msgs::CameraInfo depth_info_tmp;
  sensor_msgs::CameraInfo rgb_info_tmp;

  ros::Subscriber pc_subscriber;
  Cloud::Ptr cloud;
  rosbag::Bag bag;
  size_t every_th_frame;
  int message_index;

  bool iterate_dir_;
  bool msgs_left;
  bool pc_recieved;
  bool sync_completed;
  boost::mutex mutex;
  boost::shared_ptr<rosbag::View> view_ptr_;
  rosbag::View::iterator view_it_;
  rosbag::View::iterator view_it_end_;


};

#endif /* BAGHANDLER_H_ */
