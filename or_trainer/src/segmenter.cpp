/*
 * segmenter.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: raitalaama
 */

#include "or_trainer/segmenter.h"

namespace or_training
{
using std::vector;
using helper_functions::getFloatSettings;
using helper_functions::getIntSettings;
Segmenter::Segmenter() :
    nh_("or_trainer/segmenter")
{
  // TODO Auto-generated constructor stub
  loadSettings();
}

Segmenter::~Segmenter()
{
  // TODO Auto-generated destructor stub
}

void Segmenter::segment()
{
  ROS_INFO("Starting to segment point cloud");
  std::string meta;
  meta = input_pipeline_ + "_" + input_class_;
  meta_output_.push_back(meta);

  if (perform_initial_pass_through_filtering_)
  {
    initialPassThrough();
  }
  else
  {
    output_.push_back(input_);
  }

  if (perform_crcb_skin_filtering_)
  {
    crCbSkinFiltering();
  }

  if (perform_table_top_filtering_)
  {
    tableTopFiltering();
  }
  if (perform_final_pass_through_filtering_)
  {
    finalPassThrough();
  }
  if (perform_euclidean_clustering_)
  {
    euclideanClustering();
  }
}

void Segmenter::setInputCloud(Cloud::Ptr input)
{
  input_ = input;
}

void Segmenter::setInputClass(std::string input_class)
{
  input_class_ = input_class;
}

void Segmenter::setInputPipeline(std::string input_pipeline)
{
  input_pipeline_ = input_pipeline;
}

std::vector<std::string> Segmenter::getResultMetaData()
{
  return meta_output_;
}

std::vector<Cloud::Ptr> Segmenter::getResultPointCloud()
{
  return output_;
}

void Segmenter::initialPassThrough()
{
  ROS_INFO("initial pass through");
  std::vector<Cloud::Ptr> tmp_output;
  std::vector<std::string> tmp_meta_output;
  Cloud::Ptr processed_cloud;
  for (size_t i = 0; i < initial_pass_through_x_.size(); ++i)
  {

    initial_pass_through_x_[i].setInputCloud(input_);
    processed_cloud.reset(new Cloud);
    initial_pass_through_x_[i].filter(*processed_cloud);
    initial_pass_through_y_[i].setInputCloud(processed_cloud);
    processed_cloud.reset(new Cloud);
    initial_pass_through_y_[i].filter(*processed_cloud);
    initial_pass_through_z_[i].setInputCloud(processed_cloud);
    processed_cloud.reset(new Cloud);
    initial_pass_through_z_[i].filter(*processed_cloud);
    tmp_output.push_back(processed_cloud);

    //Meta data for screen shot filename etc.
    float x_min, x_max, y_min, y_max, z_min, z_max;
    initial_pass_through_x_[i].getFilterLimits(x_min, x_max);
    initial_pass_through_y_[i].getFilterLimits(y_min, y_max);
    initial_pass_through_z_[i].getFilterLimits(z_min, z_max);
    std::ostringstream new_meta;
    new_meta.precision(3);
    new_meta << meta_output_[0];
    new_meta << "_ipt_x_" << x_min << "_" << x_max;
    new_meta << "_y_" << y_min << "_" << y_max;
    new_meta << "_z_" << z_min << "_" << z_max;
    tmp_meta_output.push_back(new_meta.str());

    //TODO read parameter to save or not
    saveImage(processed_cloud, new_meta.str());
  }

  output_ = tmp_output;
  meta_output_ = tmp_meta_output;
}

void Segmenter::finalPassThrough()
{
}

void Segmenter::crCbSkinFiltering()
{
  ROS_INFO("crcb filtering");

  std::vector<Cloud::Ptr> tmp_output;
  std::vector<std::string> tmp_meta_output;
  Cloud::Ptr processed_cloud;

  for (size_t i = 0; i < output_.size(); ++i)
  {

    for (size_t j = 0; j < crcb_skin_model_.size(); ++j)
    {
      processed_cloud.reset(new Cloud);
      processed_cloud = output_[i];
      filterSkin(crcb_skin_model_[j], processed_cloud);
      tmp_output.push_back(processed_cloud);

      //Meta data for screen shot filename etc.
      std::ostringstream new_meta;
      new_meta.precision(3);
      new_meta << meta_output_[i];
      new_meta << crcb_skin_parameters_[j];
      tmp_meta_output.push_back(new_meta.str());

      //TODO if
      saveImage(processed_cloud, new_meta.str());
    }
  }
  output_ = tmp_output;
  meta_output_ = tmp_meta_output;

}

void Segmenter::euclideanClustering()
{
  ROS_INFO("Euclidean clustering");

  std::vector<Cloud::Ptr> tmp_output;
  std::vector<std::string> tmp_meta_output;
  Cloud::Ptr processed_cloud;
  for (size_t i = 0; i < output_.size(); ++i)
  {

    for (size_t j = 0; j < euclidean_clustering_.size(); ++j)
    {
      processed_cloud.reset(new Cloud);
      processed_cloud = output_[i];
      std::vector<pcl::PointIndices> cluster_indices;
      saveImage(processed_cloud, "timber");

      std::cout <<"org "<<processed_cloud->isOrganized()<<"\n";
      pcl::search::OrganizedNeighbor<PointRGB>::Ptr nn(new pcl::search::OrganizedNeighbor<PointRGB>);
      nn->setInputCloud(processed_cloud);
      euclidean_clustering_[j].setIndices(helper_functions::getFiniteIndices(processed_cloud));
      euclidean_clustering_[j].setSearchMethod(nn);

      euclidean_clustering_[j].setInputCloud(processed_cloud);
      euclidean_clustering_[j].extract(cluster_indices);

      pcl::ExtractIndices<PointRGB> extract;
      extract.setKeepOrganized(true);
      extract.setInputCloud(processed_cloud);

      // Create pcl::PointIndices::Ptr, first of the indices should be largest and the object we want
      extract.setIndices(boost::make_shared<pcl::PointIndices>(cluster_indices.at(0)));
      processed_cloud.reset(new Cloud);
      extract.filter(*processed_cloud);

      std::ostringstream new_meta;
      new_meta.precision(3);
      new_meta << meta_output_[i];
      new_meta << "_ec_tol_" << euclidean_clustering_[j].getClusterTolerance() << "_sz_"
          << euclidean_clustering_[j].getMinClusterSize() << "_" << euclidean_clustering_[j].getMaxClusterSize();

      tmp_output.push_back(processed_cloud);
      tmp_meta_output.push_back(new_meta.str());

      saveImage(processed_cloud, new_meta.str());

    }
  }
  output_ = tmp_output;
  meta_output_ = tmp_meta_output;
}

void Segmenter::tableTopFiltering()
{
}

bool Segmenter::isSkin(const cv::Mat skin_model, const cv::Vec3b color)
{
  using namespace cv;
  Mat input = Mat(Size(1, 1), CV_8UC3, (Scalar)(color));

  Vec3b ycrcb = input.at<Vec3b>(0, 0);
  return ((skin_model.at<uchar>(ycrcb[1], ycrcb[2]) > 0));
}

//TODO make own class, inherit  filter class?
void Segmenter::filterSkin(const cv::Mat skin_model, Cloud::Ptr &cloud)
{
  using namespace cv;

  Mat imgBGR = helper_functions::pcTocv(cloud, "getSkinColoredPoints");
  Mat imgYCrCb;
  Mat binary_mask = Mat(cloud->height, cloud->width, CV_8UC1, Scalar(0));
  pcl::PointIndices::Ptr skin_indices(new pcl::PointIndices);

  cvtColor(imgBGR, imgYCrCb, CV_BGR2YCrCb);

  MatIterator_<Vec3b> img_it = imgYCrCb.begin<Vec3b>();
  MatIterator_<uchar> bin_it = binary_mask.begin<uchar>();
  MatIterator_<Vec3b> img_end = imgYCrCb.end<Vec3b>();
  for (; img_it != img_end; ++img_it, ++bin_it)
  {
    if (isSkin(skin_model, *img_it))
    {
      *bin_it = 255;
    }
  }
  /*  namedWindow("binary_mask", CV_WINDOW_AUTOSIZE);
   imshow("binary_mask", binary_mask);
   waitKey();*/
  Mat element = cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)); //TODO make to use parameters

  cv::erode(binary_mask, binary_mask, element);
  /* namedWindow("binary_mask eroded", CV_WINDOW_AUTOSIZE);
   imshow("binary_mask eroded", binary_mask_modified);
   waitKey();*/
  cv::dilate(binary_mask, binary_mask, element, Point(2, 2), 3); //TODO setting for iterations
  /*  namedWindow("binary_mask dilated", CV_WINDOW_AUTOSIZE);
   imshow("binary_mask dilated", binary_mask);
   waitKey();*/
  int i = 0;
  bin_it = binary_mask.begin<uchar>();
  MatIterator_<uchar> bin_it_end = binary_mask.end<uchar>();

  for (; bin_it != bin_it_end; ++i, ++bin_it)
  {
    if (*bin_it == 255)
      skin_indices->indices.push_back((i));

  }

  if (skin_indices->indices.size() > 0)
  {
    pcl::ExtractIndices<PointRGB> extract;
    extract.setKeepOrganized(true);
    extract.setNegative(true);
    Cloud::Ptr tmp;
    tmp.reset(new Cloud);
    extract.setInputCloud(cloud);
    /*  if (((int)std::count_if(indices->indices.begin(), indices->indices.end(), notZero)) == 0)
     return input_cloud;*/

    extract.setIndices(skin_indices);


    extract.filter(*cloud);
    skin_indices_.reset(new pcl::PointIndices);
    //skin_indices_ = extract.getIndices();



    //HelperFunctions::pcTocv(processed_cloud);
    // points << "Skin: " << skinIndices->indices.size() << " ";
    // points << "NonSkin: " << processed_cloud_->points.size() << " ";
  }
  else
  {
    ROS_INFO("No skin found");
  }

  /*
   namedWindow("binary_mask_modified", CV_WINDOW_AUTOSIZE);
   imshow("binary_mask_modified", binary_mask_modified);
   // waitKey();
   *
   */
//ROS_INFO("skin size %d", skinIndices->indices.size());
}

void Segmenter::loadSettings()
{
  settings_loaded_ = true;
  nh_.param("perform_initial_passthrough_filtering", perform_initial_pass_through_filtering_, false);
  nh_.param("perform_final_passthrough_filtering", perform_final_pass_through_filtering_, false);
  nh_.param("perform_CrCb_filtering", perform_crcb_skin_filtering_, false);
  nh_.param("perform_euclidean_clustering", perform_euclidean_clustering_, false);
  nh_.param("perform_table_top_filtering", perform_table_top_filtering_, false);

  std::string png_dir;
  std::string png_key;

  fh_.setFilenameExtension("png");
  if (nh_.searchParam("png_output_dir", png_key))
  {
    nh_.getParam(png_key, png_dir);
    //ROS_INFO("kansio: %s", png_dir.c_str());
    fh_.setOutputDirectory(png_dir);
  }
  else
  {
    //TODO if no dir found, use current dir
    ROS_INFO("no directory to save png found, going to crash when trying to save png");
  }
  if (perform_initial_pass_through_filtering_)
  {
    vector<float> x_min = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_x_min");
    vector<float> x_max = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_x_max");
    vector<float> y_min = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_y_min");
    vector<float> y_max = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_y_max");
    vector<float> z_min = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_z_min");
    vector<float> z_max = getFloatSettings(nh_.getNamespace() + "/pass_through/initial_z_max");

    pcl::PassThrough<PointRGB> filter_x;
    pcl::PassThrough<PointRGB> filter_y;
    pcl::PassThrough<PointRGB> filter_z;

    filter_x.setFilterFieldName("x");
    filter_y.setFilterFieldName("y");
    filter_z.setFilterFieldName("z");

    filter_x.setKeepOrganized(true);
    filter_y.setKeepOrganized(true);
    filter_z.setKeepOrganized(true);

    // check that all settings are of same length
    ROS_ASSERT(
        x_min.size() == x_max.size() && x_min.size() == y_min.size() && x_min.size() == y_max.size()
            && x_min.size() == z_min.size() && x_min.size() == z_max.size());

    for (size_t i = 0; i < x_min.size(); ++i)
    {
      filter_x.setFilterLimits(x_min[i], x_max[i]);
      filter_y.setFilterLimits(y_min[i], y_max[i]);
      filter_z.setFilterLimits(z_min[i], z_max[i]);

      initial_pass_through_x_.push_back(filter_x);
      initial_pass_through_y_.push_back(filter_y);
      initial_pass_through_z_.push_back(filter_z);
    }
  }
  if (perform_crcb_skin_filtering_)
  {
    vector<float> ellipse_center_x = getFloatSettings(
        nh_.getNamespace() + "/skin_color_CrCb_filtering/ellipse_center_x");
    vector<float> ellipse_center_y = getFloatSettings(
        nh_.getNamespace() + "/skin_color_CrCb_filtering/ellipse_center_y");
    vector<float> ellipse_axis_x = getFloatSettings(nh_.getNamespace() + "/skin_color_CrCb_filtering/ellipse_axis_x");
    vector<float> ellipse_axis_y = getFloatSettings(nh_.getNamespace() + "/skin_color_CrCb_filtering/ellipse_axis_y");
    vector<float> ellipse_angle = getFloatSettings(nh_.getNamespace() + "/skin_color_CrCb_filtering/ellipse_angle");
    erode_dilate_mask_size_ = getIntSettings(nh_.getNamespace() + "/skin_color_CrCb_filtering/erode_dilate_mask_size");
    //TODO also assert mask size
    ROS_ASSERT(
        ellipse_center_x.size() == ellipse_center_y.size() && ellipse_center_x.size() == ellipse_axis_x.size()
            && ellipse_center_x.size() == ellipse_axis_y.size() && ellipse_center_x.size() == ellipse_angle.size());

    for (size_t i = 0; i < ellipse_center_x.size(); ++i)
    {
      cv::Mat skin_model = cv::Mat::zeros(256, 256, CV_8UC1);
      cv::ellipse(skin_model, cv::Point(ellipse_center_x[i], ellipse_center_y[i]),
                  cv::Size(ellipse_axis_x[i], ellipse_axis_y[i]), ellipse_angle[i], 0.0, 360.0,
                  cv::Scalar(255, 255, 255), -1);
      crcb_skin_model_.push_back(skin_model);
      std::ostringstream skin_param;
      skin_param.precision(3);
      skin_param << "_crcb_ctr_" << ellipse_center_x[i] << "_" << ellipse_center_y[i] << "_ax_" << ellipse_axis_x[i]
          << "_" << ellipse_axis_y[i] << "_ang_" << ellipse_angle[i];
      crcb_skin_parameters_.push_back(skin_param.str());
    }
  }

  if (perform_euclidean_clustering_)
  {
    vector<float> tolerance = getFloatSettings(nh_.getNamespace() + "/euclidean_clustering/tolerance");
    vector<int> min_size = getIntSettings(nh_.getNamespace() + "/euclidean_clustering/min_size");
    vector<int> max_size = getIntSettings(nh_.getNamespace() + "/euclidean_clustering/max_size");

    pcl::EuclideanClusterExtraction<PointRGB> euclidean_clustering;

    ROS_ASSERT(tolerance.size() == min_size.size() && tolerance.size() == max_size.size());
    for (size_t i = 0; i < tolerance.size(); ++i)
    {
      euclidean_clustering.setClusterTolerance(tolerance[i]);
      euclidean_clustering.setMaxClusterSize(max_size[i]);
      euclidean_clustering.setMinClusterSize(min_size[i]);
      euclidean_clustering_.push_back(euclidean_clustering);
    }

  }

  if (!settings_loaded_)
    return;

}

void Segmenter::saveImage(Cloud::Ptr cloud, std::string meta_info)
{
  cv::Mat processed_image = helper_functions::insertText(helper_functions::pcTocv(cloud), meta_info);
  fh_.saveImage(processed_image, meta_info);
}
} /* namespace or_training */

