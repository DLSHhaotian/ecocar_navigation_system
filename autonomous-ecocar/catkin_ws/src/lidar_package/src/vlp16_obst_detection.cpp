// DEBUG INCLUDES - START
#include <iostream>

// DEBUG INCLUDES - END

#include "ros/ros.h"
// PointCloud libs
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// Odometry msg
#include "std_msgs/Float32MultiArray.h"
// For Rviz Markers
#include <visualization_msgs/Marker.h>
// LidDAR libs
#include "lidar_package/classes/Odometry.h"
#include <lidar_package/vlp16_importer.h> 
#include <lidar_package/point_handler.h>
#include <lidar_package/point_plotter.h> 
#include <lidar_package/obstacle_tracker.h> 
// Message types
#include "lidar_package/cloudsAndPlane.h"
#include "lidar_package/cloud.h"
#include "lidar_package/point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"

// Node alive service
#include <std_srvs/SetBool.h>

#define LOOP_RATE 20 

#define PRINT_DEBUG true

#define ODO_TRANSFORM true
// Voxel filter
#define APPLY_VOXEL_FILTER true
#define VOXEL_LEAF_SIZE 0.1f
// Ground Plane Requirements 
//   - Use graph ground detection (Else only RANSAC)
#define USE_GRAPH_GROUND_DETECTION true
#define FIT_PLANE_TOO true // Fit plane to found gorund points
#define USE_RANSAC_TOO true && FIT_PLANE_TOO// To remove extra points using plane
#define FIT_MAX_GROUND_PLANE_DISTANCE 0.05f
#define MAX_GROUND_PLANE_DISTANCE 0.15f
#define GROUND_PLANE_RANSAC_ITERATIONS 30

  //   - RANSAC
#if USE_GRAPH_GROUND_DETECTION == false
  #define MAX_GROUND_PLANE_DISTANCE 0.25f
  #define GROUND_PLANE_RANSAC_ITERATIONS 50
#endif
// Segmentation
#define MAX_CLOUD_DISTANCE 1.5f
// Obstacle Requirements 
#define MIN_OBSTACLE_HIGHT 0.4
#define MIN_OBSTACLE_GOUND_DISTANCE -0.7
// Hull
#define PROJECT_ONTO_CURRENT_GROUND_PLANE false // Else onto world Z-plane
#define FILTER_PROJECTION true
#define FILTER_PROJECTION_GRID_SIZE 0.2f

#define TRIM_TIME 2.0 // Max age of hulls before discarded

#define UPDATE_TYPE_BY_VISIBILITY false
#define SHOW_VISIBILITY_LINE false

//   - Plot
#define MAKE_VISUALIZATION_PLOTS true
#define KEEP_PREVIOUS_HULLS false
#define KEEP_PUBLISHING true
// Publish messages with obsts
#define PUBLISH_OBSTS true
#define PUBLISH_CLOUDS_AND_PLANE true // Requires FIT_PLANE_TOO to be true

enum NodeState {
    NODE_STATE_FATAL_ERROR = -1, // For unrecoverable errors
    NODE_STATE_OK,               // When OK
    NODE_STATE_WAIT              // Node is not ready or trying to recover from an error
};

NodeState state;

// Recieve
bool recieved_new_flag;
ros::Time recieved_new_timestamp;

// Import and ground detection inits
pcl::PointCloud<pcl::PointXYZ>::Ptr laserpoints (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr floorpoints (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr floorpoints_2nd (new pcl::PointCloud<pcl::PointXYZ>);
Plane floor_plane;

// Segmentation inits
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > flat_clouds;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_hulls;
std::vector<Extreme> cloud_extremes;

// Hulls and obstacles inits
uint32_t inital_id = 1;
uint32_t * obstacle_id_top = &inital_id;
std::vector< Hull::ptr > hulls;
std::vector< ObstacleHull::ptr > obstacles;

// Odometry inits
Odometry odo;
pcl::PointCloud<pcl::PointXYZ> pathPoints;
pcl::PointCloud<pcl::PointXYZ> carPoint;

// For plotting
int max_hulls = 0;
int max_obstacles = 0;
std::vector< int > cloud_color_ids;

pcl::PointCloud<pcl::PointXYZ>::Ptr visibility_points (new pcl::PointCloud<pcl::PointXYZ>); 



void recieveLiDARDataCallback(const velodyne_msgs::VelodyneScan &pkt)
{
  // Set timestamps
  recieved_new_timestamp = ros::Time::now();
  ROS_INFO("LiDAR data recieved!");
  laserpoints->clear();
  floorpoints->clear();
  floorpoints_2nd->clear();
  

  if(USE_GRAPH_GROUND_DETECTION){ 
    Graph::ptr graph(new Graph());
    if(ODO_TRANSFORM){
      graph->importNodesFromScan(pkt, &odo, &recieved_new_timestamp);
    }else{
      graph->importNodesFromScan(pkt);
    }
    graph->exportObstaclePoints(laserpoints, floorpoints);

    // Downsample 
    if(APPLY_VOXEL_FILTER) laserpoints = applyVoxelFilter(laserpoints, VOXEL_LEAF_SIZE);

    // Run RANSAC too
    //    Find floor
    if(FIT_PLANE_TOO) {
      floorpoints = applyVoxelFilter(floorpoints, VOXEL_LEAF_SIZE);
      floor_plane = RANSAC(floorpoints, FIT_MAX_GROUND_PLANE_DISTANCE, GROUND_PLANE_RANSAC_ITERATIONS, MOST_FITS);
    }
    //    Split floor
    if(USE_RANSAC_TOO) floor_plane.filterDistance(laserpoints, MAX_GROUND_PLANE_DISTANCE, floorpoints_2nd); 

  }else{
    // Import points
    if(ODO_TRANSFORM){
      importPackage(pkt, laserpoints, &odo, &recieved_new_timestamp); 
    }else{
      importPackage(pkt, laserpoints);
    }

    // Downsample 
    if(APPLY_VOXEL_FILTER) laserpoints = applyVoxelFilter(laserpoints, VOXEL_LEAF_SIZE);

    // Find floor
    floor_plane = RANSAC(laserpoints, MAX_GROUND_PLANE_DISTANCE, GROUND_PLANE_RANSAC_ITERATIONS, MOST_FITS);

    //Split floor
    floor_plane.filterDistance(laserpoints, MAX_GROUND_PLANE_DISTANCE, floorpoints); 
  }

  // Segmentation
  clouds.clear();
  clouds = segmentation(laserpoints, MAX_CLOUD_DISTANCE, &cloud_extremes);
  if(PRINT_DEBUG) ROS_INFO("Made %d segments", (int) clouds.size());

    // For cloud colors
  cloud_color_ids.clear();
  cloud_color_ids.resize(clouds.size());

    // Make hulls
  flat_clouds.clear();
  cloud_hulls.clear();
  hulls.clear();

  for (size_t i = 0; i < clouds.size(); i++)
  {
    if(clouds.at(i)->size() >= 3){ // Need four points to make Hull
      if(USE_GRAPH_GROUND_DETECTION || cloud_extremes.at(i).z_max - cloud_extremes.at(i).z_min > MIN_OBSTACLE_HIGHT || cloud_extremes.at(i).z_max >= MIN_OBSTACLE_GOUND_DISTANCE){ // Filter before hulls are made
        pcl::PointCloud<pcl::PointXYZ>::Ptr flat_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        // Project points onto plane
        if(!USE_GRAPH_GROUND_DETECTION && PROJECT_ONTO_CURRENT_GROUND_PLANE){
          flat_cloud = projectPoints(clouds.at(i), &floor_plane);
        }else{
          flat_cloud = projectPoints(clouds.at(i));
        }

        // Downsample projected cloud
        if(FILTER_PROJECTION && !PROJECT_ONTO_CURRENT_GROUND_PLANE) flat_cloud = applyGridFilter(flat_cloud, FILTER_PROJECTION_GRID_SIZE); 
        if(FILTER_PROJECTION && PROJECT_ONTO_CURRENT_GROUND_PLANE) flat_cloud = applyVoxelFilter(flat_cloud, FILTER_PROJECTION_GRID_SIZE); 

        flat_clouds.push_back(flat_cloud);

        hulls.push_back(Hull::ptr (new Hull(flat_cloud, true)));
      }else{
        // TODO: Treat as ignored segment
        cloud_color_ids.at(i) = -2; // Set color id: white
      }
    }else{
      // TODO: Treat as outlier
      cloud_color_ids.at(i) = -1; // Set color id: red
    }
  }

  //Pair new hulls with obstacles
  if(PRINT_DEBUG) ROS_INFO("New hulls: %d", (int) hulls.size());

  ros::Time ts_merge = ros::Time::now();

  // Trim old hulls by time
  for (auto &obstacle : obstacles) obstacle->timeTrim(TRIM_TIME);
  // Remove empty objects from obstacle list
  obstacles.erase(std::remove_if(
    obstacles.begin(), obstacles.end(),
    [](ObstacleHull::ptr& obstacle) { 
        return obstacle->isEmpty(); // put your condition here
    }), obstacles.end());
  if(PRINT_DEBUG) ROS_INFO("Obstacles after trim: %d", (int) obstacles.size());

  // Remake merged hulls for obstacles
  for (auto &obstacle : obstacles) obstacle->remakeHull();

  // Pair hulls and obstacles
  pairHullsToObstacles(&obstacles, &hulls, obstacle_id_top, &cloud_color_ids);
  if(PRINT_DEBUG) ROS_INFO("Obstacles after pairing: %d", (int) obstacles.size());

  // Remake merged hulls for obstacles
  for (auto &obstacle : obstacles) obstacle->remakeHull();

  // std::cout << ros::Time::now() - ts_merge << std::endl;


  if(UPDATE_TYPE_BY_VISIBILITY){
    if(SHOW_VISIBILITY_LINE) visibility_points->clear();
    // Update visibility
    std::vector<ObstaclePoint> obstacle_points = makeObstaclePoints(&obstacles);
    sortObstaclePointsByAngle(&obstacle_points);
    int size_of_obstacle_points = obstacle_points.size();
    setPivot(ObstaclePoint(odo.x_pos,odo.y_pos));

    int inital_point_index = findClosestObstaclePoint(&obstacle_points);
    updatePivotSegment(obstacle_points.at(inital_point_index));

    std::vector<ObstacleSegment> segments;
    addNextObstacleSegmentsFromPoint(&obstacle_points.at(inital_point_index), &segments);

    int current_point_index = inital_point_index + 1;
    if(current_point_index == size_of_obstacle_points) current_point_index = 0;
    while(current_point_index != inital_point_index){
      ObstaclePoint * current_point = &obstacle_points.at(current_point_index);    
      addNextObstacleSegmentsFromPoint(current_point, &segments);

      //  Remove ended segments
      segments.erase(std::remove_if(segments.begin(), segments.end(),
        [current_point](ObstacleSegment& segment) { 
            return segment.end == * current_point;
        }), segments.end());

      updatePivotSegment(* current_point);

      //  Find front segment
      int best_index = -1;
      float best_distance_sqr = 10001.0; // TODO: Do this in a better way
      for (int i = (segments.size() - 1); i >= 0; i--)
      {
        float distance_sqr = pivotSegment.distanceToCrossingSqr(&segments.at(i));
        if(distance_sqr < 1.0 || distance_sqr > 10000.0){ // remove parallel and close segment
          if(distance_sqr < 1.0 && distance_sqr > 0.25) segments.at(i).start.obstacle->type = 1; // Still mark as visible is close
          segments.erase(segments.begin() + i);
          best_index--;
          continue;
        }else if(distance_sqr < best_distance_sqr){
          best_distance_sqr = distance_sqr;
          best_index = i;
        } 
      }
      //  For setting type 
      if(best_index >= 0) segments.at(best_index).start.obstacle->type = 1;
      if(best_index >= 0 && SHOW_VISIBILITY_LINE) visibility_points->push_back(pivotSegment.crossPoint(&segments.at(best_index)));

      //  Cirular loop
      current_point_index++;
      if(current_point_index == size_of_obstacle_points) current_point_index = 0;
    }
  }
  
  // Flag publisher
  recieved_new_flag = true; 

  // Display time-duration of recieve
  ros::Duration recieveDone = ros::Time::now() - recieved_new_timestamp;
  if(PRINT_DEBUG && recieveDone.toSec() > 0.05) std::cout << "******************** WARNING ********************" << std::endl;
  if(PRINT_DEBUG) std::cout << "Duration of recieve: " << recieveDone << std::endl;

}
 
void recieveOdoDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_data){
  odo.update(
    msg_data->data[0], // x
    msg_data->data[1], // y
    msg_data->data[2], // z
    msg_data->data[3], // orientation
    msg_data->data[4], // speed
    msg_data->data[5] // acceleration
  );
}

bool aliveCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(!req.data) {
        ROS_INFO("Shutdown triggered by service!");
        std::exit(EXIT_SUCCESS);
    }
    res.success = (state == NODE_STATE_OK);
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vlp16_obst_detection"); 

  ros::NodeHandle n; 
  
  // Subscribe on LiDAR data
  ros::Subscriber sub_lidar = n.subscribe("velodyne_packets", 1000, recieveLiDARDataCallback);
  // Subscribe on odometry
  ros::Subscriber sub_odo = n.subscribe("car_pose_estimate", 10, recieveOdoDataCallback);

  // Publish for rviz
  ros::Publisher pub = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("point_clouds", 1);
  ros::Publisher pub_test = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("z_projections", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pub_test2 = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("car_pos", 1);

  // Publish clouds for gate classifier
  ros::Publisher pub_segmented_clouds = n.advertise< lidar_package::cloudsAndPlane > ("clouds_and_plane", 1);

  // Publish obstacles
  ros::Publisher pub_obsts = n.advertise< lidar_package::obsts >("obstacle_hulls", 10);

  ros::ServiceServer service = n.advertiseService("vlp16_obst_detection_alive", aliveCallback);

  ros::Rate loop_rate(LOOP_RATE);

  ROS_INFO("VLP-16 Obstacle detection node started");

  while (ros::ok())
  {
    if(recieved_new_flag || KEEP_PUBLISHING){
      // Set timestamps
      ros::Time timestamp = ros::Time::now();

      if(MAKE_VISUALIZATION_PLOTS){
        // Plot path
        pathPoints.push_back(pcl::PointXYZ(odo.x_pos, odo.y_pos, odo.z_pos));
        marker_pub.publish(makeLineStrip(&pathPoints, 1.0, 0.5, 0, "path", 1, false, 0.02));

        pcl::PointCloud<pcl::PointXYZ>::Ptr carPoint (new pcl::PointCloud<pcl::PointXYZ>);
        carPoint->header.frame_id = "visualization_frame";
        carPoint->push_back(pcl::PointXYZ(odo.x_pos, odo.y_pos, odo.z_pos));
        pub_test2.publish(carPoint);

        if(!USE_GRAPH_GROUND_DETECTION || FIT_PLANE_TOO){
          // Plot floor grid
          pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>);
          grid_points->push_back(floor_plane.getXYPoint(5.0 + odo.x_pos,5.0 + odo.y_pos));
          grid_points->push_back(floor_plane.getXYPoint(5.0 + odo.x_pos,-5.0 + odo.y_pos));
          grid_points->push_back(floor_plane.getXYPoint(-5.0 + odo.x_pos,5.0 + odo.y_pos));
          grid_points->push_back(floor_plane.getXYPoint(-5.0 + odo.x_pos,-5.0 + odo.y_pos));
          marker_pub.publish(makeGrids(&(* grid_points), 11, 1, 0.5 ,0.5, 0.01, "floor_plane"));
        }
        // Plot removed ground
        marker_pub.publish(makePoints(&(* floorpoints), 0.0, 0.0, 0.0, "floor_points", 1, 0.4, 0.1));
        // Plot 2nd removed ground
        if(USE_RANSAC_TOO) marker_pub.publish(makePoints(&(* floorpoints_2nd), 0.0, 0.0, 0.0, "floor_points_2nd", 1, 0.2, 0.1));

        // Plot bounding boxes for cloud segments
        pcl::PointCloud<pcl::PointXYZ>::Ptr extreme_grids(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr floor_point_grids(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < cloud_extremes.size(); i++)
        {
          if(clouds.at(i)->size() > 2){
            if(!USE_GRAPH_GROUND_DETECTION && (cloud_extremes.at(i).z_max - cloud_extremes.at(i).z_min <= MIN_OBSTACLE_HIGHT && cloud_extremes.at(i).z_max < MIN_OBSTACLE_GOUND_DISTANCE)){
              makeBoxPoints(&cloud_extremes.at(i), floor_point_grids);
            }
            else{
              makeBoxPoints(&cloud_extremes.at(i), extreme_grids);
            }
          }
        }
        marker_pub.publish(makeGrids(&(*extreme_grids), 3, 0.2, 1.0 , 0.2, 0.03, "enclosing_boxes"));
        marker_pub.publish(makeGrids(&(*floor_point_grids), 3, 0.2, 0.2 , 1.0, 0.02, "floor_boxes"));

        // Plot segmented clouds
        pub.publish(colorClouds(&clouds, &cloud_color_ids));
        pub_test.publish(colorClouds(&flat_clouds)); // projected versions

        // Plot new hulls
        static int hull_counter = 0;
        if(max_hulls <  hulls.size()) max_hulls = (int) hulls.size();
        for (size_t i = 0; i < max_hulls; i++)
        {
          if(i < hulls.size() && hulls.at(i)->vertices->size() >= 3){
            marker_pub.publish(makeLineStrip(&(* hulls.at(i)->vertices), 1.0, 0, 0, "new_hulls",(int) i + hull_counter, true));
          }else{
            marker_pub.publish(makeLineStrip(0, 1.0, 0, 0, "new_hulls", (int) i + hull_counter, true)); // To delete any previoulsy created markers
          }
        }
        if(KEEP_PREVIOUS_HULLS && recieved_new_flag) hull_counter += max_hulls; // Must be 

        // Plot bounding boxes for obstacle hulls
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_grids(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < obstacles.size(); i++)
        {
          makeBoxPoints(&obstacles.at(i)->merged_hull.extreme, hull_grids);
        }
        marker_pub.publish(makeGrids(&(*hull_grids), 3, 0.2, 1.0 , 0.2, 0.03, "obstacle_hull_boxes"));

        // Plot obstacles hulls
        static int obstacles_counter = 0;
        if(max_obstacles <  obstacles.size()) max_obstacles = (int) obstacles.size();
        for (size_t i = 0; i < max_obstacles; i++)
        {
          if(i < obstacles.size() && obstacles.at(i)->merged_hull.vertices->size() >= 3){
            if(obstacles.at(i)->type == 0 || !UPDATE_TYPE_BY_VISIBILITY) marker_pub.publish(makeLineStrip(&(* obstacles.at(i)->merged_hull.vertices), 0, 1.0, 0, "object_hulls", (int) i + obstacles_counter + 100000, true));
            else marker_pub.publish(makeLineStrip(&(* obstacles.at(i)->merged_hull.vertices), 0, 0, 1.0, "object_hulls", (int) i + obstacles_counter + 100000, true));
          }else{
            marker_pub.publish(makeLineStrip(0, 0, 1.0, 0, "object_hulls", (int) i + obstacles_counter + 100000, true)); // To delete any previoulsy created markers
          }
        }
        if(KEEP_PREVIOUS_HULLS && recieved_new_flag) obstacles_counter += max_obstacles; // Must be 
      
        if(SHOW_VISIBILITY_LINE) marker_pub.publish(makeLineStrip(&(* visibility_points), 1.0, 0.5, 0, "visible_points")); 
      }


      // Publish obstacles (TODO: Add recieved_new_flag to condiation to only publish on new data)
      if(PUBLISH_OBSTS) pub_obsts.publish(obstaclesToMsg(&obstacles, &odo, recieved_new_timestamp));
      // Publish clouds for gate classifier
      if(FIT_PLANE_TOO && PUBLISH_CLOUDS_AND_PLANE && recieved_new_flag) pub_segmented_clouds.publish(cloudsAndPlaneToMsg(&clouds, &floor_plane, &odo, recieved_new_timestamp));

      // Display time-duration of publish
      ros::Duration publishDone = ros::Time::now() - timestamp;
      if(PRINT_DEBUG && (!KEEP_PUBLISHING || recieved_new_flag)) std::cout << "Duration of publish: " << publishDone << std::endl << std::endl;

      recieved_new_flag = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}