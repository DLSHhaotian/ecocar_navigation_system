#include "Point.h"

float distanceSqr(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2){
  float x = p_1.x - p_2.x;
  float y = p_1.y - p_2.y;
  return x*x + y*y;
}
float distance(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2){
  return sqrt(distanceSqr(p_1, p_2));
}
pcl::PointXYZ getMiddlePoint(pcl::PointXYZ &p_1, pcl::PointXYZ &p_2){
  float x = (p_1.x + p_2.x) / 2.0;
  float y = (p_1.y + p_2.y) / 2.0;
  float z = (p_1.z + p_2.z) / 2.0;
  return pcl::PointXYZ(x, y, z);
}