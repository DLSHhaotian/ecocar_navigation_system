#ifndef POINT_HANDLER_h
#define POINT_HANDLER_h

#include "ros/ros.h"
#include <cmath>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "lidar_package/CONSTANTS.h"

#include "lidar_package/classes/Gate.h"
#include "lidar_package/classes/Plane.h"
#include "lidar_package/classes/Extreme.h"
#include "lidar_package/classes/Odometry.h" 


// For voxel filter
#include <pcl/filters/voxel_grid.h>
// For grid filter
#include <pcl/filters/grid_minimum.h>
// For projection and hull
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
// KD Tree and euclidian segmentation
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// Message types
#include "lidar_package/cloudsAndPlane.h"
#include "lidar_package/cloud.h"
#include "lidar_package/point.h"

#define DEBUG false

enum RANSAC_method{
  BEST_FIT,
  MOST_FITS,
  NORMAL_VECTOR_FIT,
  COLUMN_FIT
};

// RANSAC
extern Eigen::MatrixXf linear_regression_fit(Eigen::MatrixXf X, Eigen::VectorXf Y);
extern Plane RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points, const float max_dist, const int max_i = 100, RANSAC_method method = MOST_FITS, Plane * plane = NULL);
// Segmentation
extern std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist, std::vector<Extreme> * output_dest = 0);
extern std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > segmentation_old(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist);

extern void printCloud(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * clouds);

// Filters
extern pcl::PointCloud<pcl::PointXYZ>::Ptr applyVoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float leafsize);
extern pcl::PointCloud<pcl::PointXYZ>::Ptr applyGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, float leafsize);
extern pcl::PointCloud<pcl::PointXYZ>::Ptr projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, Plane * plane = 0);
extern std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > kdTreeSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, float max_neighbor_dist);

extern lidar_package::cloud cloudToMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
extern lidar_package::cloudsAndPlane cloudsAndPlaneToMsg(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * clouds, Plane * plane, Odometry * odo, ros::Time timestamp);

// Vector / point calculations
extern float vectorLength(pcl::PointXYZ * p);
extern float distanceBetweenPoint(pcl::PointXYZ * p1, pcl::PointXYZ * p2);
extern pcl::PointXYZ cross(pcl::PointXYZ * a, pcl::PointXYZ * b);
extern pcl::PointXYZ normalize(pcl::PointXYZ * p);

// Gate Classification
pcl::PointCloud<pcl::PointXYZ>::Ptr findCircleCenters(pcl::PointXYZ * p1, pcl::PointXYZ * p2, float R, Plane * plane);
extern Gate gateRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float R, float raw_max_iterations, int max_iterations, Plane * plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_full = 0);


#endif