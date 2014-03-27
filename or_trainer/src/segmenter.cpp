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
using or_common::MetaData;

//TODO use only indices instead of copying pc:s around

Segmenter::Segmenter()
{
  // TODO Auto-generated constructor stub
  loadSettings();
}

Segmenter::Segmenter(std::string node_namespace) :
    nh_(node_namespace + "/segmenter")
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
  if (indices_.size() == 0)
  {
    indices_.push_back(helper_functions::getFiniteIndices(input_cloud_));
  }
  while (meta_.size() != indices_.size())
  {
    if (meta_.size() > indices_.size())
    {
      meta_.pop_back();
    }
    else
    {
      MetaData meta;
      meta.addMeta("pipeline", input_pipeline_);
      meta.addMeta("class", input_class_);
      meta_.push_back(meta);
    }

  }
  if (perform_initial_pass_through_filtering_)
  {
    initialPassThrough();
  }
  else
  {
    //output_.push_back(input_);
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
  input_cloud_ = input;

}

void Segmenter::setInputClass(std::string input_class)
{
  input_class_ = input_class;
}

void Segmenter::setInputPipeline(std::string input_pipeline)
{
  input_pipeline_ = input_pipeline;
}

std::vector<MetaData> Segmenter::getResultMetaData()
{
  return meta_;
}

std::vector<pcl::PointIndices::Ptr> Segmenter::getResultIndices()
{
  return indices_;
}

/*
 std::vector<Cloud::Ptr> Segmenter::getResultPointCloud()
 {
 std::vector<Cloud::Ptr> output_clouds;
 for()
 return output_;
 }
 */
void Segmenter::initialPassThrough()
{
  ROS_INFO("initial pass through");
  std::vector<pcl::PointIndices::Ptr> indices_tmp;
  std::vector<MetaData> meta_tmp;
  for (size_t i = 0; i < indices_.size(); ++i)
  {
    for (size_t j = 0; j < initial_pass_through_x_.size(); ++j)
    {

      MetaData new_meta(meta_[i]);
      pcl::PointIndices::Ptr remaining_points(new pcl::PointIndices(*indices_[i]));
      saveImage(input_cloud_, remaining_points);

      initial_pass_through_x_[j].setInputCloud(input_cloud_);
      initial_pass_through_x_[j].setIndices(remaining_points);
      initial_pass_through_x_[j].filter(remaining_points->indices);
      initial_pass_through_y_[j].setInputCloud(input_cloud_);
      initial_pass_through_y_[j].setIndices(remaining_points);
      initial_pass_through_y_[j].filter(remaining_points->indices);
      initial_pass_through_z_[j].setInputCloud(input_cloud_);
      initial_pass_through_z_[j].setIndices(remaining_points);
      initial_pass_through_z_[j].filter(remaining_points->indices);
      indices_tmp.push_back(remaining_points);
      //Meta data for screen shot filename etc.

      float x_min, x_max, y_min, y_max, z_min, z_max;
      initial_pass_through_x_[j].getFilterLimits(x_min, x_max);
      initial_pass_through_y_[j].getFilterLimits(y_min, y_max);
      initial_pass_through_z_[j].getFilterLimits(z_min, z_max);
      new_meta.addMeta("init_x_pass", x_min, x_max);
      new_meta.addMeta("init_y_pass", y_min, y_max);
      new_meta.addMeta("init_z_pass", z_min, z_max);
      meta_tmp.push_back(new_meta);

      //TODO read parameter to save or not
      saveImage(input_cloud_, remaining_points, new_meta);
    }
  }
  indices_ = indices_tmp;
  meta_ = meta_tmp;
}

void Segmenter::finalPassThrough()
{
}

void Segmenter::crCbSkinFiltering()
{
  ROS_INFO("crcb filtering");

  std::vector<pcl::PointIndices::Ptr> indices_tmp;
  std::vector<MetaData> meta_tmp;

  for (size_t i = 0; i < indices_.size(); ++i)
  {
    for (size_t j = 0; j < crcb_skin_model_.size(); ++j)
    {

      pcl::PointIndices::Ptr remaining_points(new pcl::PointIndices(*indices_[i]));
      saveImage(input_cloud_, remaining_points);

      remaining_points = filterSkin(crcb_skin_model_[j], remaining_points);
      indices_tmp.push_back(remaining_points);

      //Meta data for screen shot filename etc.
      MetaData new_meta(meta_[i]);
      new_meta.addMeta(crcb_skin_parameters_[j]);
      meta_tmp.push_back(new_meta);
      //TODO if
      saveImage(input_cloud_, remaining_points, new_meta);
    }
  }
  indices_ = indices_tmp;
  meta_ = meta_tmp;

}

void Segmenter::euclideanClustering()
{
  ROS_INFO("Euclidean clustering");

  std::vector<pcl::PointIndices::Ptr> indices_tmp;
  std::vector<MetaData> meta_tmp;
  pcl::search::OrganizedNeighbor<PointRGB>::Ptr nn(new pcl::search::OrganizedNeighbor<PointRGB>);
  nn->setInputCloud(input_cloud_);
  for (size_t i = 0; i < indices_.size(); ++i)
  {
    for (size_t j = 0; j < euclidean_clustering_.size(); ++j)
    {

      IndicesPtr remaining_points(new vector<int>(indices_[i]->indices));
      vector<pcl::PointIndices> clusters;
      std::cout << "indeksei: " << remaining_points->size() << "\n";
      //TODO if setting says so
      saveImage(input_cloud_, *remaining_points);

      euclidean_clustering_[j].setIndices(remaining_points);
      euclidean_clustering_[j].setSearchMethod(nn);

      euclidean_clustering_[j].setInputCloud(input_cloud_);
      euclidean_clustering_[j].extract(clusters);
      indices_tmp.push_back(boost::make_shared<pcl::PointIndices>(clusters.at(0)));
      MetaData new_meta(meta_[i]);
      new_meta.addMeta("euc_cluster_tolerance", static_cast<float>(euclidean_clustering_[j].getClusterTolerance()));
      new_meta.addMeta("euc_cluste_size", euclidean_clustering_[j].getMinClusterSize(),
                       euclidean_clustering_[j].getMaxClusterSize());
      meta_tmp.push_back(new_meta);
      saveImage(input_cloud_, boost::make_shared<pcl::PointIndices>(clusters.at(0)));
    }
  }
  indices_ = indices_tmp;
  meta_ = meta_tmp;

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
pcl::PointIndices::Ptr Segmenter::filterSkin(const cv::Mat skin_model, pcl::PointIndices::Ptr indices)
{
  using namespace cv;

  Mat imgBGR = helper_functions::pcTocv(input_cloud_, indices);
  Mat imgYCrCb;
  Mat binary_mask = Mat(input_cloud_->height, input_cloud_->width, CV_8UC1, Scalar(0));
  vector<int> skin_indices;

  cvtColor(imgBGR, imgYCrCb, CV_BGR2YCrCb);
  //TODO iterate only indices
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
  cv::dilate(binary_mask, binary_mask, element, Point(2, 2), 4); //TODO setting for iterations
  /*  namedWindow("binary_mask dilated", CV_WINDOW_AUTOSIZE);
   imshow("binary_mask dilated", binary_mask);
   waitKey();*/
  int i = 0;
  bin_it = binary_mask.begin<uchar>();
  MatIterator_<uchar> bin_it_end = binary_mask.end<uchar>();

  for (; bin_it != bin_it_end; ++i, ++bin_it)
  {
    if (*bin_it == 255)
      skin_indices.push_back((i));
  }
  vector<int> sorted_indices(indices->indices);
  std::sort(sorted_indices.begin(), sorted_indices.end());
  vector<int> non_skin_indices;
  std::set_difference(sorted_indices.begin(), sorted_indices.end(), skin_indices.begin(), skin_indices.end(),
                      std::back_inserter(non_skin_indices));
  pcl::PointIndices::Ptr non_skin_point_indices(new pcl::PointIndices);
  non_skin_point_indices->indices = non_skin_indices;
  return non_skin_point_indices;
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

      //save meta data on settings
      MetaData new_meta;
      new_meta.addMeta("CrCb_center", ellipse_center_x[i], ellipse_center_y[i]);
      new_meta.addMeta("CrCb_axis", ellipse_axis_x[i], ellipse_axis_y[i]);
      new_meta.addMeta("CrCb_angle", ellipse_angle[i]);
      crcb_skin_parameters_.push_back(new_meta);
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
//TODO move to helpers
void Segmenter::saveImage(Cloud::Ptr cloud, pcl::PointIndices::Ptr indices, MetaData meta_info)
{
  cv::Mat processed_image = helper_functions::insertText(helper_functions::pcTocv(cloud, indices),
                                                         meta_info.getMetaString());
  fh_.saveImage(processed_image, "segmenting");
}
void Segmenter::saveImage(Cloud::Ptr cloud, pcl::PointIndices::Ptr indices)
{
  cv::Mat processed_image = helper_functions::pcTocv(cloud, indices);
  fh_.saveImage(processed_image, "segmenting");
}
void Segmenter::saveImage(Cloud::Ptr cloud, MetaData meta_info)
{
  cv::Mat processed_image = helper_functions::insertText(helper_functions::pcTocv(cloud), meta_info.getMetaString());
  fh_.saveImage(processed_image, "segmenting");
}

void Segmenter::saveImage(Cloud::Ptr cloud)
{
  cv::Mat processed_image = helper_functions::pcTocv(cloud);
  fh_.saveImage(processed_image, "segmenting");
}

void Segmenter::saveImage(Cloud::Ptr cloud, vector<int> indices)
{
  cv::Mat processed_image = helper_functions::pcTocv(cloud, indices);
  fh_.saveImage(processed_image, "segmenting");
}

} /* namespace or_training */

