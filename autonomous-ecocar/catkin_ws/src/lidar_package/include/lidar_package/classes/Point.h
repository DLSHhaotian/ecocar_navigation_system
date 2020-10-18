#pragma once
#include <pcl/point_types.h>

float distanceSqr(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2);
float distance(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2);
pcl::PointXYZ getMiddlePoint(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2);

// class Point
// {
// };