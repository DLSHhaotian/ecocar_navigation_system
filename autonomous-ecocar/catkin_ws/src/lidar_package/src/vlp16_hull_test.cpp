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

#define LOOP_RATE 20 

#define PRINT_DEBUG true

#define ODO_TRANSFORM true
// Voxel filter
#define APPLY_VOXEL_FILTER true
#define VOXEL_LEAF_SIZE 0.1f
// Ground Plane Requirements 
#define USE_GRAPH_GROUND_DETECTION true
//   - RANSAC
#define MAX_GROUND_PLANE_DISTANCE 0.025f
#define GROUND_PLANE_RANSAC_ITERATIONS 100
// Segmentation
#define MAX_CLOUD_DISTANCE 1.0f
// Obstacle Requirements 
#define MIN_OBSTACLE_HIGHT 0.4
#define MIN_OBSTACLE_GOUND_DISTANCE -0.7
// Hull
#define PROJECT_ONTO_CURRENT_GROUND_PLANE false // Else onto world Z-plane
#define FILTER_PROJECTION true
#define FILTER_PROJECTION_GRID_SIZE 0.2f

#define TRIM_TIME 0.5 // Max age of hulls before discarded
//   - Plot
#define MAKE_VISUALIZATION_PLOTS true
#define KEEP_PREVIOUS_HULLS false
#define KEEP_PUBLISHING true
// Publish messages with obsts
#define PUBLISH_OBSTS true

// Recieve
bool recieved_new_flag;
ros::Time recieved_new_timestamp;

// Import and ground detection inits
pcl::PointCloud<pcl::PointXYZ>::Ptr laserpoints (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr floorpoints (new pcl::PointCloud<pcl::PointXYZ>);
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

// Odometry inits
Odometry odo;
pcl::PointCloud<pcl::PointXYZ> pathPoints;

// For plotting
int max_hulls = 0;
std::vector< int > cloud_color_ids;


void recieveLiDARDataCallback(const velodyne_msgs::VelodyneScan &pkt)
{
  // Set timestamps
  recieved_new_timestamp = ros::Time::now();
  ros::Time temp_timestamp = ros::Time::now();

  laserpoints->clear();
  floorpoints->clear();

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

  // Display time-duration of import and floor separation (and downsample)
  std::cout << "Duration of import and floor separation: " << ros::Time::now() - temp_timestamp << std::endl;
  temp_timestamp = ros::Time::now();

  // Segmentation
  clouds.clear();
  clouds = segmentation(laserpoints, MAX_CLOUD_DISTANCE, &cloud_extremes);
  if(PRINT_DEBUG) ROS_INFO("Made %d segments", (int) clouds.size());

  // Display time-duration of import to segmentation
  std::cout << "Duration of to segmentation: " << ros::Time::now() - temp_timestamp << std::endl;
  temp_timestamp = ros::Time::now();

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
      // Only ignore flat clouds of not using graph ground removal
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

        //ROS_INFO("Of %d points, hull contains %d points", (int) flat_cloud->size(), (int) result->size());
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

  // Display time-duration of projecting and making hulls
  std::cout << "Duration of projecting and making hulls: " << ros::Time::now() - temp_timestamp << std::endl;
  temp_timestamp = ros::Time::now();
  
  // Display time-duration of recieve
  ros::Duration recieveDone = ros::Time::now() - recieved_new_timestamp;
  if(PRINT_DEBUG && recieveDone.toSec() > 0.05) std::cout << "******************** WARNING ********************" << std::endl;
  if(PRINT_DEBUG) std::cout << "Duration of recieve: " << recieveDone << std::endl;

  // Flag publisher
  recieved_new_flag = true; 


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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vlp16_hull_test"); 

  ros::NodeHandle n; 
  
  // Subscribe on LiDAR data
  // ros::Subscriber sub_lidar = n.subscribe("sim/velodyne_points", 1000, recieveLiDARDataCallback);
  ros::Subscriber sub_lidar = n.subscribe("velodyne_packets", 1000, recieveLiDARDataCallback); 
  // Subscribe on odometry
  ros::Subscriber sub_odo = n.subscribe("car_pose_estimate", 10, recieveOdoDataCallback);

  // Publish for rviz
  ros::Publisher pub = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("point_clouds", 1);
  ros::Publisher pub_test = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("z_projections", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok())
  {
    if(recieved_new_flag || KEEP_PUBLISHING){
      // Set timestamps
      ros::Time timestamp = ros::Time::now();

      if(MAKE_VISUALIZATION_PLOTS){
        // Plot path
        pathPoints.push_back(pcl::PointXYZ(odo.x_pos, odo.y_pos, odo.z_pos));
        marker_pub.publish(makeLineStrip(&pathPoints, 1.0, 0.5, 0, "path"));

       if(!USE_GRAPH_GROUND_DETECTION){ 
          // Plot floor grid 
          pcl::PointCloud<pcl::PointXYZ>::Ptr grid_points(new pcl::PointCloud<pcl::PointXYZ>); 
          grid_points->push_back(floor_plane.getXYPoint(10.0,10.0)); 
          grid_points->push_back(floor_plane.getXYPoint(10.0,-10.0)); 
          grid_points->push_back(floor_plane.getXYPoint(-10.0,10.0)); 
          grid_points->push_back(floor_plane.getXYPoint(-10.0,-10.0)); 
          marker_pub.publish(makeGrids(&(* grid_points), 20, 1, 0.5 ,0.5, 0.01, "floor_plane")); 
        } 
        
        // Plot removed ground
        marker_pub.publish(makePoints(&(* floorpoints), 1.0, 1.0, 1.0, "floor_points", 1, 0.2, 0.1));

        // Plot bounding boxes for cloud segments
        pcl::PointCloud<pcl::PointXYZ>::Ptr extreme_grids(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr floor_point_grids(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < cloud_extremes.size(); i++)
        {
          if(clouds.at(i)->size() > 2){
            if(cloud_extremes.at(i).z_max - cloud_extremes.at(i).z_min <= MIN_OBSTACLE_HIGHT && cloud_extremes.at(i).z_max < MIN_OBSTACLE_GOUND_DISTANCE){
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
        if(KEEP_PREVIOUS_HULLS) hull_counter += max_hulls; // Must be 
      }
      
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