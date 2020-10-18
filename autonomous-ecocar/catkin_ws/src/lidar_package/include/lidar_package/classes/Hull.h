#pragma once
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include "Extreme.h"
#include "Point.h"

#define USE_OPT_SWINGING_ARM true
#define SWINGING_ARM_LENGT 1.5 // [m]
#define CONCAVE_HULL_ALPHA 0 // 0 for convex hulls (equal to inf alpha)

#define COMPARE_HULL_MIN_FITS 1 // but to hull size if less
#define COMPARE_HULL_MAX_DISTANCE 1.0 // [m]

class Hull
{
  private:
    float distToLineSegmentOnZPlane(pcl::PointXYZ * p, pcl::PointXYZ * l_1, pcl::PointXYZ * l_2);
    float distToLineSegmentOnZPlaneSqr(pcl::PointXYZ * p, pcl::PointXYZ * l_1, pcl::PointXYZ * l_2);
    bool doSegmentsCross(pcl::PointXYZ * p1, pcl::PointXYZ * p2, pcl::PointXYZ * p3, pcl::PointXYZ * p4);
  public:
    typedef std::shared_ptr<Hull> ptr;
    ros::Time timestamp;
    Extreme extreme;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices;
    Hull();
    Hull(pcl::PointCloud<pcl::PointXYZ>::Ptr in_vertices, bool make_hull = false, Extreme in_extreme = Extreme(), ros::Time in_timestamp = ros::Time::now());
    Hull(ptr other_hull);
    void updateExtreme();
    Hull copyHull();
    int size();
    void clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr makeHull(pcl::PointCloud<pcl::PointXYZ>::Ptr planar_projection_cloud);
    bool compareToHull(Hull::ptr other_hull);
    void printHull();

    pcl::PointCloud<pcl::PointXYZ>::Ptr newSwingingArmAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r);
    pcl::PointCloud<pcl::PointXYZ>::Ptr swingingArmAlgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float r);
};