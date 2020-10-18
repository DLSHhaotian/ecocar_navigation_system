// DEBUG INCLUDES - START
#include <iostream>
#include "lidar_package/classes/MovingAverage.h"
#include "lidar_package/classes/Gate.h"
#include "lidar_package/classes/TrackedGate.h"
#include "lidar_package/classes/GateTracker.h"

// DEBUG INCLUDES - END

#include "ros/ros.h"
// PointCloud libs
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// For Rviz Markers
#include <visualization_msgs/Marker.h>
// LidDAR libs
#include <lidar_package/point_handler.h>
#include <lidar_package/point_plotter.h> 
// Message types
#include "lidar_package/cloudsAndPlane.h"
#include "lidar_package/cloud.h"
#include "lidar_package/point.h"
#include "lidar_package/gates.h"

#define LOOP_RATE 20
#define DEBUG_GATE false
#define DEBUG_PRINT_TIME true
#define PLOT_GATE_DATA false

ros::Time recieved_new_timestamp;
bool recieved_new_flag = false;
Plane floor_plane;

// Many clouds since memory is cheaper than processingtime
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds_flat;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds_flat_full;

pcl::PointCloud<pcl::PointXYZ>::Ptr gate_columns (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr gate_centers (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_colored_gate_centers (new pcl::PointCloud<pcl::PointXYZRGB>); //Debug

// Global car position for each received scan
float x_position = 0, y_position = 0, z_position = 0, orientation = 0;
ros::Time package_time;

GateTracker gate_tracker(4, 7);


void recievedCloudsAndPlaneCallback(const lidar_package::cloudsAndPlane &pkt)
{
  const int MIN_POINTS_PER_CLOUD = 4;
  const float REQUIRED_DISTANCE_FROM_GROUND_PLANE = 0.6; // [m] Used to make horizontal slice for detecting areas of interest
  const float MAX_DISTANCE_FROM_GROUND_PLANE = 1.1;
  const float GATE_RADIUS = 0.225; // [m]
  const float GATE_RANSAC_MAX_POINT_DISTANCE = 0.1; // [m] The tolerance or threshold for filtering out outliers for the model 
  const int CIRCLE_RANSAC_ITERATIONS = 50;
  const float MAX_GATE_DISTANCE = 1.0; // [m] Maximal distance between gates, in case more gates are part of the same cloud 
  const float FULL_INCLUSION_DISTANCE = 0.1; // [m] extra distance from extremes to include 'full' points

  recieved_new_flag = true;
  if(DEBUG_PRINT_TIME){
    recieved_new_timestamp = ros::Time::now();
  }

  // Get car position at sensing
  x_position = pkt.x;
  y_position = pkt.y;
  z_position = pkt.z;
  orientation = pkt.orientation;
  // Get floor plane
  floor_plane = Plane(pkt.a, pkt.b, pkt.c, pkt.d);
  // Get timestamp from package
  package_time = pkt.timestamp;


  if(DEBUG_GATE){
    ROS_INFO("Recieved Data");
    ROS_INFO("a, b, c, d = %f, %f, %f, %f", pkt.a, pkt.b, pkt.c, pkt.d);
    ROS_INFO("x, y, z = %f, %f, %f", pkt.x, pkt.y, pkt.z);
    ROS_INFO("Time: %f", package_time.toSec());
    ROS_INFO("Number of clouds: %d", (int) pkt.vector_len);
  }

  // Extract pointclouds
  clouds.clear();
  clouds_flat.clear();
  clouds_flat_full.clear();
  for (size_t cloud_index = 0; cloud_index < pkt.vector_len; cloud_index++)
  {
    if(DEBUG_GATE) ROS_INFO("\tPoints in cloud %d: %d", (int) cloud_index, (int) pkt.vector_cloud.at(cloud_index).vector_len);
    int cloud_size = pkt.vector_cloud.at(cloud_index).vector_len;

    if(cloud_size < MIN_POINTS_PER_CLOUD) continue; // Skip if cloud is too small

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_flat (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_flat_full (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t point_index = 0; point_index < cloud_size; point_index++)
    {
      // Get point
      pcl::PointXYZ point(
        pkt.vector_cloud.at(cloud_index).vector_point.at(point_index).x,
        pkt.vector_cloud.at(cloud_index).vector_point.at(point_index).y,
        pkt.vector_cloud.at(cloud_index).vector_point.at(point_index).z
      );

      pcl::PointXYZ projected_point = floor_plane.projectOntoPlane(&point);
      float distance_from_plane = floor_plane.DistToPoint(&point);

      // Filter horizontal slice (also filteres from Full Cloud)
      if(distance_from_plane > MAX_DISTANCE_FROM_GROUND_PLANE) continue; // Skip if too far from plane

      // Add to Full Cloud
      temp_cloud_flat_full->push_back(projected_point);
      
      // Filter horizontal slice
      if(distance_from_plane <= REQUIRED_DISTANCE_FROM_GROUND_PLANE) continue; // Skip if too close to plane
      if(floor_plane.getZ(point.x, point.y) > point.z) continue; // Skip if under plane in respect to z
      
      // Add to Filtered Cloud
      temp_cloud->push_back(point);
      temp_cloud_flat->push_back(projected_point);

    }

    // Split clouds if multiple gate prospects can be found (if too much distance between points)
    std::vector<Extreme> cloud_extremes;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > splitted_cloud = segmentation(temp_cloud_flat, MAX_GATE_DISTANCE, &cloud_extremes);
    if(DEBUG_GATE) ROS_INFO("\t\tFound %d clouds!", (int) splitted_cloud.size()); 
    for (size_t splitted_cloud_index = 0; splitted_cloud_index < splitted_cloud.size(); splitted_cloud_index++)
    {
      if(splitted_cloud.at(splitted_cloud_index)->size() >= MIN_POINTS_PER_CLOUD){
        if(DEBUG_GATE) ROS_INFO("\t\t\tadding cloud"); 
        clouds.push_back(temp_cloud);
        clouds_flat.push_back(splitted_cloud.at(splitted_cloud_index));
        clouds_flat_full.push_back(cloud_extremes.at(splitted_cloud_index).filterNearXY(temp_cloud_flat_full, FULL_INCLUSION_DISTANCE));
      }else{
        if(DEBUG_GATE) ROS_INFO("\t\t\tskiping cloud (too small)"); 
      }
    }
  }

  // Find gates
  if(DEBUG_GATE){
    ROS_INFO("Find gates...");
    test_colored_gate_centers->clear();
  }
  gate_centers->clear();
  gate_columns->clear();
  for (size_t cloud_index = 0; cloud_index < clouds_flat.size(); cloud_index++)
  {
    if(DEBUG_GATE) ROS_INFO("\t\tPoints in final flat cloud %d: %d (Full cloud: %d)", (int) cloud_index, (int) clouds_flat.at(cloud_index)->size(), (int) clouds_flat_full.at(cloud_index)->size());
    Gate gatePole = gateRANSAC(
      clouds_flat.at(cloud_index),
      GATE_RADIUS, 
      GATE_RANSAC_MAX_POINT_DISTANCE,
      CIRCLE_RANSAC_ITERATIONS, 
      &floor_plane, 
      clouds_flat_full.at(cloud_index)
    );
    if(!gatePole.isSet()){
      if(DEBUG_GATE){
        ROS_INFO("\t\t\tNo gate found!");
      }
    } else {
      gate_tracker += gatePole;
    }
  }
  gate_tracker.update();
  gate_tracker.exportGatesToPoints(gate_columns);
  gate_tracker.findGatePairs(gate_centers);
  // Display time-duration of recieve
  if(DEBUG_PRINT_TIME){
    ros::Duration recieveDone = ros::Time::now() - recieved_new_timestamp;
    std::cout << "Duration of recieve: " << recieveDone << std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gate_classifier"); 

  ros::NodeHandle n; 

  // Subscribe clouds for gate classifier
  ros::Subscriber sub_segmented_clouds = n.subscribe("clouds_and_plane", 1, recievedCloudsAndPlaneCallback);
  // Publish found gates
  ros::Publisher pub_gates = n.advertise< lidar_package::gates > ("gates", 1);

  // For Rviz visualization
  ros::Publisher pub_clouds = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("gate_clouds", 1);
  ros::Publisher pub_clouds_flat = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("gate_clouds_flat", 1);
  ros::Publisher pub_clouds_flat_full = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("gate_clouds_flat_full", 1);
  ros::Publisher pub_gate_columns = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("gate_columns", 1);
  ros::Publisher pub_gate_centers = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("gate_centers", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate loop_rate(LOOP_RATE);

  ROS_INFO("Gate classification node started");

  while (ros::ok())
  {
    if(recieved_new_flag){
      ROS_INFO("Publish gates...");
      pub_gates.publish(gate_tracker.gatesToMsg(x_position, y_position, z_position, orientation, package_time));
      recieved_new_flag = false;
    }
    if(PLOT_GATE_DATA){
      // Plot clouds
      pub_clouds.publish(colorClouds(&clouds));
      pub_clouds_flat.publish(colorClouds(&clouds_flat));
      pub_clouds_flat_full.publish(colorClouds(&clouds_flat_full));
      // Plot floor grid
      pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);
      grid_points->push_back(floor_plane.getXYPoint(5.0 + x_position,5.0 + y_position));
      grid_points->push_back(floor_plane.getXYPoint(5.0 + x_position,-5.0 + y_position));
      grid_points->push_back(floor_plane.getXYPoint(-5.0 + x_position,5.0 + y_position));
      grid_points->push_back(floor_plane.getXYPoint(-5.0 + x_position,-5.0 + y_position));
      marker_pub.publish(makeGrids(&(* grid_points), 11, 1, 0.5 ,0.5, 0.01, "floor_plane_gate"));
    } 

    // Plot gates (only centers)
    gate_columns->header.frame_id = "visualization_frame";
    pub_gate_columns.publish(gate_columns);
    gate_centers->header.frame_id = "visualization_frame";
    pub_gate_centers.publish(gate_centers);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}