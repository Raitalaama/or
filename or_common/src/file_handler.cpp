/*
 * file_handler.cpp
 *
 *  Created on: Jun 7, 2012
 *      Author: risto
 */

#include "or_common/file_handler.h"

using namespace std;
using namespace pcl;

using namespace boost::filesystem;

FileHandler::FileHandler()
{
  // TODO Auto-generated constructor stub
  loop_input_files = false;

  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

}
FileHandler::~FileHandler()
{
  // TODO Auto-generated destructor stub
}

void FileHandler::setLoopInput(bool loop_files)
{
  loop_input_files = loop_files;
}

void FileHandler::saveImage(const cv::Mat image)
{
  //TODO Crashes if input is empty point cloud

  try
  {
    cv::imwrite(getCompleteOutputFilename(), image, compression_params);
  }
  catch (runtime_error& ex)
  {
    fprintf(stderr, "Exception saving image to PNG format: %s\n", ex.what());
  }
}

void FileHandler::savePointCloud(Cloud::ConstPtr cloud)
{
  //TODO Crashes if input is empty point cloud
  //(output_pcd_directory.string())

  io::savePCDFileASCII(getCompleteOutputFilename(), *cloud);
}

void FileHandler::savePointCloud(Cloud::ConstPtr cloud, const PointIndices::ConstPtr &indices)
{
  Cloud::Ptr temp;
  temp.reset(new Cloud);
  temp = HelperFunctions::filterIndices(cloud, indices, true);
  savePointCloud(temp);
}

void FileHandler::savePointCloud(Cloud::ConstPtr cloud, std::vector<PointIndices> &indice_vector)
{
  for (vector<PointIndices>::iterator it = indice_vector.begin(); it != indice_vector.end(); ++it)
    savePointCloud(cloud, boost::make_shared<const PointIndices>(*it));
  //TODO save all from this function call with 1 particular prefix
}

//TODO use only constcloudptr for parameters?

Cloud::Ptr FileHandler::getNextPointCloud()
{
  Cloud::Ptr cloud;
  cloud.reset(new Cloud);
  sensor_msgs::PointCloud2 cloud_blob;

  if (loop_input_files && files_iterator_ == files_.end())
    files_iterator_ = files_.begin();
  if (files_iterator_ != files_.end())
  {
    if (io::loadPCDFile((*files_iterator_).string(), cloud_blob) == -1) //* load the file
    {
      //printf ("Couldn't read file %s \n",pcd_iterator->string());
    }
    ROS_INFO("height of the read pc %u", cloud_blob.height);
    fromROSMsg(cloud_blob, *cloud);
    ++files_iterator_;
    //TODO maybe get "cloud" as input parameter instead (with void return)
  }
  else
    printf("No (more) .pcd files in the directory");
  cloud->header.frame_id = "/camera_depth_optical_frame";
  return cloud;
}

void FileHandler::saveTrainingData(const cv::Mat features, const cv::Mat classes)
{
  if (filename_extension_ != "yml" && filename_extension_ != "yaml" && filename_extension_ != "xml")
  {
    ROS_INFO("invalid filename extension, aborting save (accepted extensions: yml,yaml,xml");
    return;
  }
  if (features.rows != classes.rows)
  {
    ROS_INFO("Number of feature vectors and classes doesn't match, aborting saving");
    return;
  }
  std::cout << getCompleteOutputFilename() << "\n";
  cv::FileStorage fs(getCompleteOutputFilename(), cv::FileStorage::WRITE);
  fs << "Feature vectors" << features << "Classes" << classes;
  fs.release();
}

void FileHandler::saveFeatures(const cv::Mat data)
{
  if (filename_extension_ != "yml" && filename_extension_ != "yaml" && filename_extension_ != "xml")
  {
    ROS_INFO("invalid filename extension, aborting save (accepted extensions: yml,yaml,xml");
    return;
  }
  std::cout << getCompleteOutputFilename() << "\n";
  cv::FileStorage fs(getCompleteOutputFilename(), cv::FileStorage::WRITE);
  fs << "Feature vectors" << data;
  fs.release();
}

void FileHandler::saveTrainingData(const std::vector<cv::Mat> features, const std::vector<cv::Mat> classes)
{
  if (features.size() != classes.size())
  {
    ROS_INFO("Number of feature types and classes doesn't match, aborting saving");
    return;
  }
  for (size_t i = 0; i < features.size(); ++i)
    saveTrainingData(features.at(i), classes.at(i));
}

void FileHandler::loadAnnotatedFeaturesDir(std::vector<cv::Mat>* features, std::vector<cv::Mat>* classes)
{
  loadInputFiles();
  if (filename_extension_ != "yml" && filename_extension_ != "yaml" && filename_extension_ != "xml")
  {
    ROS_INFO("invalid filename extension, aborting loading (accepted extensions: yml,yaml,xml");
    return;
  }
  if (files_iterator_ == files_.end())
  {
    ROS_INFO("No files with current extension in the current directory");
    return;
  }

  while (files_iterator_ != files_.end())
  {
    cv::FileStorage fs((*files_iterator_).string(), cv::FileStorage::READ);
    //std::cout<<fs.isOpened()<<"\n";
    //std::cout<<(*files_iterator_).string();
    cv::Mat feature_vectors, feature_classes;
    fs["Classes"] >> feature_classes;
    fs["Feature vectors"] >> feature_vectors;

//      if (feature_classes.cols > 0&& feature_classes.rows > 0 )
//            std::cout<<" class:"<< feature_classes.at<int>(0, 0)<<"\n";
    features->push_back(feature_vectors);
    classes->push_back(feature_classes);
    //std::cout <<"Inside loader: "<<feature_vectors.rows<<" "<<feature_classes.rows<<"\n";
    ++files_iterator_;
    fs.release();
  }

}

void FileHandler::loadTrainingDataDir(std::vector<cv::Mat>* features, std::vector<cv::Mat>* classes,
                                      float percentage_of_features_used)
{
  loadInputFiles();
  if (filename_extension_ != "yml" && filename_extension_ != "yaml" && filename_extension_ != "xml")
  {
    ROS_INFO("invalid filename extension, aborting loading (accepted extensions: yml,yaml,xml");
    return;
  }
  if (files_iterator_ == files_.end())
  {
    ROS_INFO("No files with current extension in the current directory");
    return;
  }

  while (files_iterator_ != files_.end())
  {
    cv::FileStorage fs((*files_iterator_).string(), cv::FileStorage::READ);
    //std::cout<<fs.isOpened()<<"\n";
    cv::Mat feature_vectors, feature_classes;
    fs["Classes"] >> feature_classes;
    fs["Feature vectors"] >> feature_vectors;
    //std::cout <<"Inside loader: "<<feature_vectors.rows<<" "<<feature_classes.rows<<"\n";
    int nr_of_vectors = 0;
    int class_iter = 0;
    cv::MatIterator_<int> classes_begin = feature_classes.begin<int>();
    cv::MatIterator_<int> classes_end = feature_classes.end<int>();

    std::map<int, int> features_per_class;
    std::map<int, int> added_features_per_class;

    while (nr_of_vectors < feature_classes.rows)
    {
      int class_count = count(classes_begin, classes_end, class_iter);
      if (class_count > 0)
      {
        nr_of_vectors += class_count;
       // std::cout<<"krsss\n";

        features_per_class[class_iter] = (int)(class_count * percentage_of_features_used);
        std::cout<<"class: "<<class_iter<<"max feats:"<<(int)(class_count * percentage_of_features_used)<<"\n";
        added_features_per_class[class_iter] = 0;
      }
      ++class_iter;
    }

    //std::cout<<added_features_per_class<<"\n"<<features_per_class;
    size_t feature_type = 0;
    while (feature_type < features->size())
    {
      if (feature_vectors.cols == features->at(feature_type).cols)
      {

        for (int row_i = 0; row_i < feature_vectors.rows; ++row_i)
        {

          if (isAllowedClass(feature_classes.row(row_i).at<int>(0))
              && added_features_per_class[feature_classes.row(row_i).at<int>(0)]
                  < features_per_class[feature_classes.row(row_i).at<int>(0)])
          {
            features->at(feature_type).push_back(feature_vectors.row(row_i));
            classes->at(feature_type).push_back(feature_classes.row(row_i));
            ++added_features_per_class[feature_classes.row(row_i).at<int>(0)];
           // std::cout<<"class: "<<feature_classes.row(row_i).at<int>(0)<<"current feats/max:"<<added_features_per_class[feature_classes.row(row_i).at<int>(0)]<<"/"<<features_per_class[feature_classes.row(row_i).at<int>(0)]<<"\n";

          }
        }
        break;
      }
      ++feature_type;
    }
    if (feature_type == features->size())
    {

      for (int row_i = 0; row_i < feature_vectors.rows; ++row_i)
              {
                if (isAllowedClass(feature_classes.row(row_i).at<int>(0))
                    && added_features_per_class[feature_classes.row(row_i).at<int>(0)]
                        < features_per_class[feature_classes.row(row_i).at<int>(0)])
                {
                  if(features->size()==0)
                  {
                  features->push_back(feature_vectors.row(row_i));
                  classes->push_back(feature_classes.row(row_i));
                 // std::cout<<"class: "<<feature_classes.row(row_i).at<int>(0)<<"current feats/max:"<<added_features_per_class[feature_classes.row(row_i).at<int>(0)]<<"/"<<features_per_class[feature_classes.row(row_i).at<int>(0)]<<"\n";
                  }
                  else
                  {
                    features->at(feature_type).push_back(feature_vectors.row(row_i));
                                classes->at(feature_type).push_back(feature_classes.row(row_i));
                  }
                  ++added_features_per_class[feature_classes.row(row_i).at<int>(0)];

                }
              }
      /*      for (int row_i = 0; row_i < feature_vectors.rows; ++row_i)
       {
       if (isAllowedClass(feature_classes.row(row_i).at<int>(0)))
       {
       features->push_back(feature_vectors.row(row_i));
       classes->push_back(feature_classes.row(row_i));
       }
       }*/
    }

    //std::cout<<features.size()<<classes.size()<<"\n";
    ++files_iterator_;
    fs.release();
  }
}

bool FileHandler::isAllowedClass(int class_name)
{

  std::vector<int>::iterator begin = non_used_classes_.begin();
  std::vector<int>::iterator end = non_used_classes_.end();
  return (std::count(begin, end, class_name) == 0);
}

void pushBackFeatures(cv::Mat feature_vector)
{

}

void FileHandler::loadTrainingData(cv::Mat features, cv::Mat classes)
{
  //TODO do i have to use pointers to features and classes?
  std::string extension = filename_extension_;
  if (extension != "yml" && extension != "yaml" && extension != "xml")
  {
    ROS_INFO("invalid filename extension, trying yml (accepted extensions: yml,yaml,xml");
    extension = "yml";
  }
  if (!exists(path(getCompleteInputFilename())))
  {
    ROS_INFO("File specified could not be found");
    return;
  }
  cv::FileStorage fs(getCompleteInputFilename(), cv::FileStorage::READ);
  fs["Feature vectors"] >> features;
  fs["Classes"] >> classes;
  fs.release();
}

