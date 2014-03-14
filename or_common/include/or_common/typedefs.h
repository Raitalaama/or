/*
 * typedefs.h
 *
 *  Created on: Jul 9, 2012
 *      Author: risto
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointRGB> Cloud;
//typedef Cloud::Ptr CloudPtr;
//typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> CloudN;
//typedef CloudN::Ptr CloudNPtr;
//typedef CloudN::ConstPtr CloudNPConstPtr;

typedef pcl::PointXYZRGBNormal PointRGBN;
typedef pcl::PointCloud<PointRGBN> CloudRGBN;
//typedef CloudXYZRGBN::Ptr CloudXYZRGBNPtr;
//typedef CloudXYZRGBN::ConstPtr CloudXYZRGBNConstPtr;


typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

#endif /* TYPEDEFS_H_ */
