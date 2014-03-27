/*
 * PointIndicesMeta.h
 *
 *  Created on: Mar 18, 2014
 *      Author: raitalaama
 */

#ifndef METADATA_H_
#define METADATA_H_

#include <vector>
#include <utility>
#include <string>
#include <sstream>
#include <ros/ros.h> //TODO ota pois

namespace or_common
{

class MetaData
{
public:
  MetaData();
  virtual ~MetaData();

  bool metaEqualsTo(MetaData other);

  void addMeta(MetaData other);

  void addMeta(std::string variable_name, std::string variable_value);
  void addMeta(std::string variable_name, int variable_value);
  void addMeta(std::string variable_name, float variable_value);
  //Meta for min max or x y etc. parameters
  void addMeta(std::string variable_name, int variable_value_min, int variable_value_max);
  void addMeta(std::string variable_name, float variable_value_min, float variable_value_max);

  std::vector<std::pair<std::string, std::string> > &getMeta();
  std::string getMetaString();
private:

  std::vector<std::pair<std::string, std::string> > meta_data_;

};

} /* namespace or_common */

#endif /* METADATA_H_ */
