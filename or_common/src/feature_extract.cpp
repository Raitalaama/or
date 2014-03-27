/*
 * FeatureExtract.cpp
 *
 *  Created on: Mar 15, 2014
 *      Author: raitalaama
 */

#include "or_common/feature_extract.h"

namespace or_common
{
using std::vector;
using helper_functions::getFloatSettings;
using helper_functions::getIntSettings;

FeatureExtract::FeatureExtract()
{
  // TODO Auto-generated constructor stub
  loadSettings();
}
FeatureExtract::FeatureExtract(std::string node_namespace) :
    nh_(node_namespace + "/feature_extractor")
{
  // TODO Auto-generated constructor stub
  loadSettings();
}

void FeatureExtract::loadSettings()
{
  have_input_indices_ = false;
  have_normals_ = false;

  //FPFH settings
  vector<float> search_radius = getFloatSettings(nh_.getNamespace() + "/fpfh_estimation/search_radius");
  for (size_t i = 0; i < search_radius.size(); ++i)
  {
    pcl::FPFHEstimationOMP<PointRGB, PointN, pcl::FPFHSignature33> fpfh_estimator;
    fpfh_estimator.setSearchMethod(
        pcl::search::OrganizedNeighbor<PointRGB>::Ptr(new pcl::search::OrganizedNeighbor<PointRGB>));
    fpfh_estimator.setRadiusSearch(search_radius[i]);
  }

  //Uniform sampling settings
  vector<float> leaf_size = getFloatSettings(nh_.getNamespace() + "/uniform_sampling/leaf_size");
  for (size_t i = 0; i < leaf_size.size(); ++i)
  {
    pcl::UniformSampling<PointRGB> uniform_sampling;
    uniform_sampling.setRadiusSearch(leaf_size[i]);
    uniform_sampling_.push_back(uniform_sampling);
  }

  //Normal computing settings
  vector<float> max_depth_change_factor = getFloatSettings(
      nh_.getNamespace() + "/normal_estimation/max_depth_change_factor");
  vector<float> smoothing_size = getFloatSettings(nh_.getNamespace() + "/normal_estimation/smoothing_size");
  vector<int> estimation_method = getIntSettings(nh_.getNamespace() + "/normal_estimation/estimation_method");

  ROS_ASSERT(
      max_depth_change_factor.size() == smoothing_size.size()
          && max_depth_change_factor.size() == estimation_method.size());
  for (size_t i = 0; i < smoothing_size.size(); ++i)
  {
    pcl::IntegralImageNormalEstimation<PointRGB, PointN> normal_estimator;
    switch (estimation_method[i])
    {
      case 1:
        normal_estimator.setNormalEstimationMethod(normal_estimator.COVARIANCE_MATRIX);
        break;
      case 2:
        normal_estimator.setNormalEstimationMethod(normal_estimator.AVERAGE_3D_GRADIENT);
        break;
      case 3:
        normal_estimator.setNormalEstimationMethod(normal_estimator.AVERAGE_DEPTH_CHANGE);
        break;
      default:
        normal_estimator.setNormalEstimationMethod(normal_estimator.COVARIANCE_MATRIX);
        ROS_INFO("Normal estimation method setting not set properly, defaulting to covariance matrix");
    }
    normal_estimator.setNormalEstimationMethod(normal_estimator.AVERAGE_3D_GRADIENT);
    normal_estimator.setMaxDepthChangeFactor(max_depth_change_factor[i]);
    normal_estimator.setNormalSmoothingSize(smoothing_size[i]);
    normal_estimator.setBorderPolicy(normal_estimator.BORDER_POLICY_MIRROR);
    pcl::search::OrganizedNeighbor<PointRGB>::Ptr neighbor(new pcl::search::OrganizedNeighbor<PointRGB>);
    normal_estimator.setSearchMethod(neighbor);
    normal_estimators_.push_back(normal_estimator);
  }

  //normal_estimator.setDepthDependentSmoothing(true);

}
FeatureExtract::~FeatureExtract()
{
  // TODO Auto-generated destructor stub
}

void FeatureExtract::setInputCloud(Cloud::Ptr input)
{
  input_cloud_ = input;
  have_input_indices_=false;
  have_normals_=false;
}

vector<CloudN::Ptr> FeatureExtract::getNormals()
{
  return normals_;
}

void FeatureExtract::setInputMeta(std::string input_meta)
{
  input_meta_ = input_meta;
}

void FeatureExtract::setInputNormals(vector<CloudN::Ptr> input_normals)
{

  normals_ = input_normals;
  have_normals_ = true;
}

void FeatureExtract::setInputIndices(pcl::PointIndices::Ptr input_indices)
{
  input_indices_ = input_indices;
  have_input_indices_ = true;
}

std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> FeatureExtract::getFPFHDescriptors()
{
  return fpfh_descriptors_;
}

std::vector<cv::Mat> FeatureExtract::getFPFHDescriptorsCV()
{
  std::vector<cv::Mat> descriptors_cv;
  for (std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr>::iterator fpfh_it = fpfh_descriptors_.begin();
      fpfh_it != fpfh_descriptors_.end(); ++fpfh_it)
  {
    cv::Mat features;
    if ((*fpfh_it)->size() > 0)
    {
      features = cv::Mat(cv::Size(33, (*fpfh_it)->points.size()), CV_32FC1);
      for (size_t i = 0; i < (*fpfh_it)->points.size(); ++i)
      {
        float* features_row = features.ptr<float>(i);
        for (size_t j = 0; j < 33; ++j)
        {
          features_row[j] = (*fpfh_it)->points[i].histogram[j];

        }

      }
    }
    descriptors_cv.push_back(features);
  }
  return descriptors_cv;
}

void FeatureExtract::computeFPFHDescriptors()
{
  if (!have_normals_)
  {
    computeNormals();
  }
  if (!keypoints_.size() > 0)
  {
    computeUniformKeypoints();
    ROS_INFO("No keypoints, calculating uniform ones");
  }

  ROS_INFO("FPFH descriptors");

  fpfh_descriptors_.clear();
  for (std::vector<CloudN::Ptr>::iterator n_it = normals_.begin(); n_it != normals_.end(); ++n_it)
  {
    for (std::vector<Cloud::Ptr>::iterator k_it = keypoints_.begin(); k_it != keypoints_.end(); ++k_it)
    {
      for (std::vector<pcl::FPFHEstimationOMP<PointRGB, PointN, pcl::FPFHSignature33> >::iterator f_it =
          fpfh_estimators_.begin(); f_it != fpfh_estimators_.end(); ++f_it)
      {
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());

        f_it->setInputCloud(*k_it);
        f_it->setInputNormals(*n_it);
        f_it->setSearchSurface(input_cloud_);
        if (have_input_indices_)
        {
          f_it->setIndices(input_indices_);
        }
        f_it->compute(*fpfh_descriptor);
        fpfh_descriptors_.push_back(fpfh_descriptor);
      }
    }

  }
}

void FeatureExtract::computeUniformKeypoints()
{
  ROS_INFO("uniform keypoints");
  for (std::vector<pcl::UniformSampling<PointRGB> >::iterator it = uniform_sampling_.begin();
      it != uniform_sampling_.end(); ++it)
  {
    Cloud::Ptr keypoints(new Cloud());
    pcl::PointCloud<int> keypoint_indices;
    it->setInputCloud(input_cloud_);
    if (have_input_indices_)
    {
      it->setIndices(input_indices_);
    }
    it->compute(keypoint_indices);
    pcl::copyPointCloud(*input_cloud_, keypoint_indices.points, *keypoints);
    keypoints_.push_back(keypoints);

  }

}
//TODO normaalit pitäs vastata inputtei
void FeatureExtract::computeNormals()
{
  ROS_INFO("normals");
  normals_.clear();
  for (std::vector<pcl::IntegralImageNormalEstimation<PointRGB, PointN> >::iterator it = normal_estimators_.begin();
      it != normal_estimators_.end(); ++it)
  {
    CloudN::Ptr normals(new CloudN());
    it->setInputCloud(input_cloud_);
    it->compute(*normals);
    normals_.push_back(normals);

    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(input_cloud_);
    viewer->addPointCloud<pcl::PointXYZRGB>(input_cloud_, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(input_cloud_, normals, 10, 0.05, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "normals");

    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }



    pcl::io::savePCDFileASCII("/home/raitalaama/or_output/test_pcd.pcd", *(input_cloud_));
    pcl::io::savePCDFileASCII("/home/raitalaama/or_output/test_normals.pcd", *normals);
*/
  }

  have_normals_ = true;

}

} /* namespace or_common */
