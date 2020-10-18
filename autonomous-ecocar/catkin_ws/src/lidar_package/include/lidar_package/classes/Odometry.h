#pragma once

#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define USE_NEAREST_POINT false // Use nearest data point insted of interpolation
#define MIN_DURATION ros::Duration(0.0001) // To avoid zero/near zero division

struct odo_data{
  float x_pos;
  float y_pos;
  float z_pos;
  float orientation;
  float speed;
  float acceleration;
  ros::Time time_of_receival;

  float cos_orientation; // For faster calculation
  float sin_orientation; // For faster calculation
};

class Odometry
{
  private:
    std::vector<odo_data> data_log;
    int32_t log_size;
    int32_t top_index; // Points at the newest data (index)
    bool log_is_full;
    
    odo_data * getData(int32_t i);
    odo_data interpolateData(odo_data * from_odo, odo_data * to_odo, ros::Time time);
  public:
    // Current (Can be removed) 
    float x_pos; 
    float y_pos; 
    float z_pos; 
    float orientation; 
    float speed; 
    float acceleration; 
    float cos_orientation; // For faster calculation 
    float sin_orientation; // For faster calculation 
    ros::Time time_of_receival; 

    Odometry(int32_t in_log_size = 10);
    void update(float in_x, float in_y, float in_z, float in_orientation, float in_speed = 0.0, float in_acceleration = 0.0);
    void transformPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);
    void transformClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * pcls);
    odo_data getInterpolatedData(ros::Time time);
    void printLog();
};
