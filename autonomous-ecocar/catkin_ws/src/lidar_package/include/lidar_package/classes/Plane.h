#pragma once

#include "lidar_package/CONSTANTS.h"
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Plane
{
  private:
        float length;
        bool is_set;
        bool is_nomalized;
  public:
        float a;
        float b;
        float c;
        float d;
        Plane();
        Plane(pcl::PointXYZ * p_1, pcl::PointXYZ * p_2, pcl::PointXYZ * p_3);
        Plane(float input_a, float input_b, float input_c);
        Plane(float input_a, float input_b, float input_c, float input_d);
        void Normalize();
        float getLength();
        pcl::PointXYZ getNormalVector();
        float DistToPoint(pcl::PointXYZ * p);
        pcl::PointXYZ projectOntoPlane(pcl::PointXYZ * p);
        bool abovePlane(pcl::PointXYZ * p);
        float SqErrToPLC(pcl::PointCloud<pcl::PointXYZ> * plc);
        void filterDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float distance, pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_output = pcl::PointCloud<pcl::PointXYZ>::Ptr());
        float getAngleToNorm(float x, float y, float z);
        float getAngleBetweenPlanes(Plane * other_plane);
        float getZ(float x, float y);
        pcl::PointXYZ getXYPoint(float x, float y);
        void clear();
        bool isSet();
};