/*
 * paramHelper.h
 *
 *  Created on: Aug 15, 2013
 *      Author: viki
 */

#ifndef PARAMHELPER_H_
#define PARAMHELPER_H_
#include "ros/node_handle.h"
class MyNodeHandle : public ros::NodeHandle
{
private:
  void printInfo(const std::string s)
  {

  }
public:
  MyNodeHandle(){};
  virtual ~MyNodeHandle(){};

  std::string getParamEx(const std::string& key, const char* val)
  {
    std::string rv;
    if (!NodeHandle::getParam(key, rv))
    {
      ROS_WARN("Param [%s] not found.", key.c_str());
      rv = val;
    }
    ROS_INFO("Set param [%s] to [%s]", key.c_str(), rv.c_str());
    return rv;
  }

  std::string getParamEx(const std::string& key, std::string val)
  {
    std::string rv;
    if (!NodeHandle::getParam(key, rv))
    {
      ROS_WARN("Param [%s] not found.", key.c_str());
      rv = val;
    }
    ROS_INFO("Set param [%s] to [%s]", key.c_str(), rv.c_str());
    return rv;
  }

  double getParamEx(const std::string& key, double d)
  {
    double rv;
    if (!NodeHandle::getParam(key, rv))
    {
      ROS_WARN("Param [%s] not found.", key.c_str());
      rv = d;
    }
    ROS_INFO("Set param [%s] to [%f]", key.c_str(), rv);
    return rv;
  }

  int getParamEx(const std::string& key, int i)
  {
    int rv;
    if (!NodeHandle::getParam(key, rv))
    {
      ROS_WARN("Param [%s] not found.", key.c_str());
      rv = i;
    }
    ROS_INFO("Set param [%s] to [%d]", key.c_str(), rv);
    return rv;
  }

  bool getParamEx(const std::string& key, bool b)
  {
    bool rv;
    if (!NodeHandle::getParam(key, rv))
    {
      ROS_WARN("Param [%s] not found.", key.c_str());
      rv = b;
    }
    ROS_INFO("Set param [%s] to [%s]", key.c_str(), rv ? "true" : "false");
    return rv;
  }
};

#endif /* PARAMHELPER_H_ */
