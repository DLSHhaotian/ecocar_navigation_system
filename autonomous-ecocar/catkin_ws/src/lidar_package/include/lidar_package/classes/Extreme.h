#pragma once
#include "lidar_package/CONSTANTS.h"

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Extreme
{
    private: 
      bool is_set;
    public:
      float x_min;
      float x_max;
      float y_min;
      float y_max;
      float z_min;
      float z_max;
      Extreme();
      Extreme(
        float input_x_min, float input_x_max, 
        float input_y_min, float input_y_max, 
        float input_z_min, float input_z_max
      );
      void clear();
      void makeExtreme(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
      bool isSet();
      bool isNearXY(pcl::PointXYZ * p, float dist);
      pcl::PointCloud<pcl::PointXYZ>::Ptr filterNearXY(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float dist);
      bool isNear(pcl::PointXYZ * p, float dist);
      bool isNear(Extreme * other_extreme, float dist);
      void update(pcl::PointXYZ * p);
      void merge(Extreme * other_extreme);
      void printExtreme();
};