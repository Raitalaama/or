/*
 * helper_functions.cpp
 *
 *  Created on: May 31, 2012
 *      Author: risto
 */

#include "or_common/helper_functions.h"

//TODO make invariant to image size, make text wrap
cv::Mat helper_functions::insertText(cv::Mat image, std::string line1, std::string line2, std::string line3,
                                     std::string line4, std::string line5, std::string line6)
{
  if (image.elemSize() == 1)
    cv::Scalar color(0);
  else
    cv::Scalar color(255, 0, 0);

  cv::putText(image, line1, cv::Point(5, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  cv::putText(image, line2, cv::Point(5, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  cv::putText(image, line3, cv::Point(5, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  cv::putText(image, line4, cv::Point(5, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  cv::putText(image, line5, cv::Point(5, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  cv::putText(image, line6, cv::Point(5, 120), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));
  return image;
}

pcl::PointIndices::Ptr helper_functions::getFiniteIndices(Cloud::ConstPtr input_cloud)
{
  pcl::PointIndices::Ptr finite_indices(new pcl::PointIndices());
  for (size_t i = 0; i < input_cloud->points.size(); ++i)
  {
    if (boost::math::isfinite(input_cloud->points[i].x)) //TODO any point in checking y and z?
    {
      finite_indices->indices.push_back(i);
    }
  }
  return finite_indices;
}



cv::Mat helper_functions::pcTocv(Cloud::Ptr &input_cloud, std::string window_name)
{

  cv::Mat image = pcTocv(input_cloud);
  //namedWindow(window_name, CV_WINDOW_AUTOSIZE);
  //imshow(window_name, image);
  //waitKey();
  return image;
}

cv::Mat helper_functions::pcToDistcv(Cloud::Ptr &input_cloud)
{
  if (input_cloud->height == 1)
    ROS_INFO("trying to convert non organized point cloud");
  using cv::Mat;
  Mat image = Mat(cv::Size(input_cloud->width, input_cloud->height), CV_8UC1, cv::Scalar(255));

  Cloud::iterator it = input_cloud->points.begin();
  Cloud::iterator end = input_cloud->points.end();

  cv::MatIterator_<uchar> img_it = image.begin<uchar>();
  float min, max;
  min = std::numeric_limits<float>::max();
  max = std::numeric_limits<float>::min();
  for (; it != end; ++it, ++img_it)
  {
    if ((boost::math::isfinite)(it->z) && (boost::math::isfinite)(it->y) && (boost::math::isfinite)(it->x))
    {
      if ((it->z) < min)
        min = it->z;
      if ((it->z) > max)
        max = it->z;
    }
  }
  it = input_cloud->points.begin();
  img_it = image.begin<uchar>();
  for (; it != end; ++it, ++img_it)
  {
    if ((boost::math::isfinite)(it->z) && (boost::math::isfinite)(it->y) && (boost::math::isfinite)(it->x))
    {
      (*img_it) = 200 - (it->z - min) * 200 / (max - min);
    }
  }
  return image;
}

cv::Mat helper_functions::pcTocv(Cloud::Ptr &input_cloud)
{
  if (input_cloud->height == 1)
    ROS_INFO("trying to convert non organized point cloud");
  cv::Mat image = cv::Mat(cv::Size(input_cloud->width, input_cloud->height), CV_8UC3, cv::Scalar(0, 0, 255));

  Cloud::iterator it = input_cloud->points.begin();
  Cloud::iterator end = input_cloud->points.end();

  cv::MatIterator_<cv::Vec3b> img_it = image.begin<cv::Vec3b>();
  for (; it != end; ++it, ++img_it)
  {
    if ((boost::math::isfinite)(it->z) && (boost::math::isfinite)(it->y) && (boost::math::isfinite)(it->x))
    {

      (*img_it).val[0] = it->b;
      (*img_it).val[1] = it->g;
      (*img_it).val[2] = it->r;
    }
  }

  return image;
}

cv::Mat helper_functions::pcTocv(Cloud::Ptr &input_cloud, pcl::PointIndices::Ptr indices)
{
  return pcTocv(input_cloud,indices->indices);
}

cv::Mat helper_functions::pcTocv(Cloud::Ptr &input_cloud, IndicesPtr indices)
{
  return pcTocv(input_cloud,*indices);
}

cv::Mat helper_functions::pcTocv(Cloud::Ptr &input_cloud, std::vector<int> indices)
{
  if (input_cloud->height == 1)
    ROS_INFO("trying to convert non organized point cloud");
  cv::Mat image = cv::Mat(cv::Size(input_cloud->width, input_cloud->height), CV_8UC3, cv::Scalar(0, 0, 255));

  std::vector<int>::iterator it = indices.begin();
  std::vector<int>::iterator end = indices.end();

  for (; it != end; ++it)
  {
    if ((boost::math::isfinite)(input_cloud->points[*it].z) && (boost::math::isfinite)(input_cloud->points[*it].y)
        && (boost::math::isfinite)(input_cloud->points[*it].x)) //TODO any need to check more than one?
    {
      image.at<cv::Vec3b>(*it).val[0] = input_cloud->points[*it].b;
      image.at<cv::Vec3b>(*it).val[1] = input_cloud->points[*it].g;
      image.at<cv::Vec3b>(*it).val[2] = input_cloud->points[*it].r;
    }
  }
  return image;
}



cv::Mat helper_functions::highlightPoints(cv::Mat base_image, boost::shared_ptr<std::vector<int> > highlight_points)
{
  const int WIDTH = base_image.cols;

  std::vector<int>::iterator it = highlight_points->begin();
  std::vector<int>::iterator end = highlight_points->end();

  for (; it != end; ++it)
  {
    int width_tmp = *it;
    int height_tmp = 0;
    while ((width_tmp) > WIDTH)
    {
      width_tmp -= WIDTH;
      ++height_tmp;
    }
    cv::circle(base_image, cv::Point(width_tmp, height_tmp), 10, cv::Scalar(238, 130, 238), 1);
    //circle(base_image,Point(width_tmp,height_tmp),1,Scalar(238,130,238),-1);
  }
  return base_image;

}

bool notZero(int i)
{
  return (i = !0);
}

std::vector<int> helper_functions::getIntSettings(std::string setting_name)
{
  std::vector<int> data;

  XmlRpc::XmlRpcValue xml_data;
  std::string key;

  if (ros::param::search(setting_name, key))
  {
    ros::param::get(key, xml_data);
    ROS_ASSERT(xml_data.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < xml_data.size(); ++i)
    {
      ROS_ASSERT(xml_data[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      data.push_back(static_cast<int>(xml_data[i]));
    }
  }
  return data;
}

std::vector<float> helper_functions::getFloatSettings(std::string setting_name)
{
  std::vector<float> data;
  XmlRpc::XmlRpcValue xml_data;
  std::string key;

  if (ros::param::search(setting_name, key))
  {
    ros::param::get(key, xml_data);
    ROS_ASSERT(xml_data.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < xml_data.size(); ++i)
    {
      ROS_ASSERT(xml_data[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      double tmp = static_cast<double>(xml_data[i]);
      data.push_back(static_cast<float>(tmp));

    }
  }

  return data;
}

std::string helper_functions::getStringSetting(std::string setting_name)
{
  cv::FileStorage fs("settings.yml", cv::FileStorage::READ);
  std::string setting = (std::string)fs[setting_name];
  fs.release();
  return setting;
}

std::vector<std::string> helper_functions::getStringVectorSetting(std::string setting_name)
{
  cv::FileStorage fs("settings.yml", cv::FileStorage::READ);
  cv::FileNode n = fs[setting_name];                         // Read string sequence - Get node

  std::vector<std::string> vector;
  if (n.type() != cv::FileNode::SEQ)
  {
    std::cerr << setting_name << " is not a sequence! Empty vector returned" << std::endl;
    return vector;
  }

  cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
  for (; it != it_end; ++it)
    vector.push_back((std::string)*it);
  fs.release();
  std::cout << "laoded vectorsize: " << vector.size() << "\n";
  return vector;
}
