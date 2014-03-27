/*
 * meta_data.cpp
 *
 *  Created on: Mar 21, 2014
 *      Author: raitalaama
 */

#include "or_common/meta_data.h"

namespace or_common
{

MetaData::MetaData():meta_data_()
{
  // TODO Auto-generated constructor stub

}

MetaData::~MetaData()
{
  // TODO Auto-generated destructor stub
}

void MetaData::addMeta(MetaData other)
{
  meta_data_.insert(meta_data_.end(), other.getMeta().begin(), other.getMeta().end());
}

void MetaData::addMeta(std::string variable_name, std::string variable_value)
{
  meta_data_.push_back(std::make_pair(variable_name, variable_value));
}
void MetaData::addMeta(std::string variable_name, int variable_value)
{
  std::ostringstream formatter;
  formatter << variable_value;
  meta_data_.push_back(std::make_pair(variable_name, formatter.str()));
}
void MetaData::addMeta(std::string variable_name, float variable_value)
{
  std::ostringstream formatter;
  formatter.scientific;
  formatter.precision(4);
  formatter << variable_value;
  meta_data_.push_back(std::make_pair(variable_name, formatter.str()));
}
//Meta for min max or x y etc. parameters
void MetaData::addMeta(std::string variable_name, int variable_value_min, int variable_value_max)
{
  std::ostringstream formatter;
  formatter << variable_value_min << "_" << variable_value_max;
  meta_data_.push_back(std::make_pair(variable_name, formatter.str()));
}
void MetaData::addMeta(std::string variable_name, float variable_value_min, float variable_value_max)
{
  std::ostringstream formatter;
  formatter.scientific;
  formatter.precision(4);
  formatter << variable_value_min << "_" << variable_value_max;
  meta_data_.push_back(std::make_pair(variable_name, formatter.str()));
}

std::vector<std::pair<std::string, std::string> >& MetaData::getMeta()
{
  return meta_data_;
}
std::string MetaData::getMetaString()
{
  std::ostringstream stream;
  for (size_t i = 0; i < meta_data_.size(); ++i)
  {
    stream << meta_data_[i].first << "_" << meta_data_[i].second << "_";
  }
  return stream.str();
}

} /* namespace or_common */
