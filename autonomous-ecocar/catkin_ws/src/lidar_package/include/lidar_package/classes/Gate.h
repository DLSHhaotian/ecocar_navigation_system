#pragma once
#include "ros/ros.h"
#include <pcl/point_types.h>

class Gate
{
  private:
    bool is_set;
    bool has_timestamp;
    ros::Time timestamp;
  public:
    typedef std::shared_ptr<Gate> ptr;
    pcl::PointXYZ center;
    float fit_fraction;
    float fit_deviation;
    int fitted_points;
    Gate();
    Gate(pcl::PointXYZ in_center, float in_fit_fraction, float in_fit_deviation, int in_fitted_points);
    void operator+=(Gate& newGate);
    Gate operator/(float divider);
    bool isSet();
    void setIsSet(bool state);
    void setTimestamp(ros::Time in_timestamp);
    bool hasTimestamp();
    ros::Time getTimestamp();
};