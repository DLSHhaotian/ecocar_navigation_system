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
// LiDAR libs
#include "lidar_package/classes/Odometry.h"
#include <lidar_package/vlp16_importer.h> 
#include <lidar_package/point_handler.h>
#include <lidar_package/point_plotter.h> 


#define LOOP_RATE 20 

#define ODO_TRANSFORM true
// Voxel filter
#define APPLY_VOXEL_FILTER false
#define VOXEL_LEAF_SIZE 0.1f
// Ground Plane Requirements 
//   - RANSAC
#define MAX_GROUND_PLANE_DISTANCE 0.2f
#define GROUND_PLANE_RANSAC_ITERATIONS 90


bool recieved_new;
Odometry odo(30);
pcl::PointCloud<pcl::PointXYZ> pathPoints;
pcl::PointCloud<pcl::PointXYZ> carPoint;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserpoints (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserpoints_colored_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserpoints_colored_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr roughness_laserpoints (new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZ>::Ptr floorpoints (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr full_laserpoints (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_laserpoints_colored_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_laserpoints_colored_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_roughness_laserpoints (new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZ>::Ptr full_floorpoints (new pcl::PointCloud<pcl::PointXYZ>);


pcl::PointCloud<pcl::PointXYZ>::Ptr verticalFromList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr verticalToList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr horizonalFromList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr horizonalToList (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr full_verticalFromList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr full_verticalToList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr full_horizonalFromList (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr full_horizonalToList (new pcl::PointCloud<pcl::PointXYZ>);


void recieveLiDARDataCallback(const velodyne_msgs::VelodyneScan &pkt)
{
    ros::Time timestamp = ros::Time::now();
    ros::Time timestamp2 = ros::Time::now();

    laserpoints->clear();
    importPackage(pkt, laserpoints, 0, &timestamp, true); 

    // Display time-duration of recieve
    ros::Duration recieveDone = ros::Time::now() - timestamp2;
    std::cout << "Duration of classic recieve: " << recieveDone << std::endl;

    // floorpoints->clear();
    // timestamp2 = ros::Time::now();
    // Downsample 
    laserpoints = applyVoxelFilter(laserpoints, VOXEL_LEAF_SIZE);

    // recieveDone = ros::Time::now() - timestamp2;
    // std::cout << "Duration of downsample: " << recieveDone << std::endl;
    // timestamp2 = ros::Time::now();

    // Find floor
    Plane floor_plane;
    floor_plane = RANSAC(laserpoints, MAX_GROUND_PLANE_DISTANCE, GROUND_PLANE_RANSAC_ITERATIONS, MOST_FITS);

    //Split floor
    floor_plane.filterDistance(laserpoints, MAX_GROUND_PLANE_DISTANCE, floorpoints); 
    
    // recieveDone = ros::Time::now() - timestamp2;
    // std::cout << "Duration of classic RANSAC: " << recieveDone << std::endl;
    // timestamp2 = ros::Time::now();


    Graph::ptr test_graph(new Graph());
    //  odo_data odo_test = odo.getInterpolatedData(timestamp);
    //  test_graph->importNodesFromPackage((Package *) &pkt.packets[0].data, &odo_test);
    test_graph->importNodesFromScan(pkt, &odo, &timestamp);

    // // Display time-duration of recieve
    // recieveDone = ros::Time::now() - timestamp2;
    // std::cout << "Duration of graph recieve: " << recieveDone << std::endl;
    // timestamp2 = ros::Time::now();


    verticalFromList->clear();
    verticalToList->clear();
    test_graph->exportVerticalToList(verticalFromList, verticalToList);

    horizonalFromList->clear();
    horizonalToList->clear();
    test_graph->exportHorizontalToList(horizonalFromList, horizonalToList);

    laserpoints_colored_1->clear();
    test_graph->exportToPoints(laserpoints_colored_1);
    
    laserpoints_colored_2->clear();
    test_graph->exportToPoints2(laserpoints_colored_2);

    // recieveDone = ros::Time::now() - timestamp2;
    // std::cout << "Duration of graph split: " << recieveDone << std::endl;
    roughness_laserpoints->clear();
    test_graph->exportToPoints3(roughness_laserpoints);

    // FULL TEST

    ros::Time full_timestamp = ros::Time::now();
    full_laserpoints->clear();
    importPackage(pkt, full_laserpoints, &odo, &full_timestamp, false); 

    // // Display time-duration of recieve
    // ros::Duration full_recieveDone = ros::Time::now() - full_timestamp;
    // std::cout << "Duration of full_classic recieve: " << full_recieveDone << std::endl;

    // full_floorpoints->clear();
    // ros::Time full_timestamp_2 = ros::Time::now();
    // // Downsample 
    // full_laserpoints = applyVoxelFilter(full_laserpoints, VOXEL_LEAF_SIZE);

    // // Find floor
    // floor_plane = RANSAC(full_laserpoints, MAX_GROUND_PLANE_DISTANCE, GROUND_PLANE_RANSAC_ITERATIONS, MOST_FITS);

    // //Split floor
    // floor_plane.filterDistance(full_laserpoints, MAX_GROUND_PLANE_DISTANCE, full_floorpoints); 
    // full_recieveDone = ros::Time::now() - full_timestamp;
    // std::cout << "Duration of full_classic RANSAC: " << full_recieveDone << std::endl;


    // full_timestamp_2 = ros::Time::now();

    Graph::ptr full_test_graph(new Graph());
    //  odo_data odo_test = odo.getInterpolatedData(timestamp);
    //  test_graph->importNodesFromPackage((Package *) &pkt.packets[0].data, &odo_test);
    full_test_graph->importNodesFromScan(pkt, &odo, &timestamp, false);

    // // Display time-duration of recieve
    // full_recieveDone = ros::Time::now() - full_timestamp;
    // std::cout << "Duration of full_graph recieve: " << full_recieveDone << std::endl;

    full_verticalFromList->clear();
    full_verticalToList->clear();
    full_test_graph->exportVerticalToList(full_verticalFromList, full_verticalToList);

    full_horizonalFromList->clear();
    full_horizonalToList->clear();
    full_test_graph->exportHorizontalToList(full_horizonalFromList, full_horizonalToList);

    // full_laserpoints_colored_1->clear();
    // full_test_graph->exportToPoints(full_laserpoints_colored_1);
    // full_laserpoints_colored_2->clear();
    // full_test_graph->exportToPoints2(full_laserpoints_colored_2);

    // full_roughness_laserpoints->clear();
    // full_test_graph->exportToPoints3(full_roughness_laserpoints);


    // Flag publisher
    recieved_new = true; 
}
 
void recieveOdoDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg_data){
    odo.update(
      msg_data->data[0], // x
      msg_data->data[1], // y
      msg_data->data[2], // z
      msg_data->data[3], // orientation
      msg_data->data[4], // speed
      msg_data->data[5]);// acceleration
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "vlp16_graph_test"); 

  ros::NodeHandle n; 
  
  // Subscribe on LiDAR data
  ros::Subscriber sub_lidar = n.subscribe("velodyne_packets", 1000, recieveLiDARDataCallback);
  // Subscribe on odometry
  ros::Subscriber sub_odo = n.subscribe("car_pose_estimate", 10, recieveOdoDataCallback);

  // Publish points
  ros::Publisher pub = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("pointcloud_test", 1);
  ros::Publisher pub_floor = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("pointcloud_test_floor", 1);
  ros::Publisher pub_test_1 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("point_clouds_test_1", 1);
  ros::Publisher pub_test_2 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("point_clouds_test_2", 1);
  ros::Publisher pub_test_3 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("point_clouds_test_roughness", 1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Publisher full_pub = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("full_pointcloud_test", 1);
  ros::Publisher full_pub_floor = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("full_pointcloud_test_floor", 1);
  ros::Publisher full_pub_test_1 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("full_point_clouds_test_1", 1);
  ros::Publisher full_pub_test_2 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("full_point_clouds_test_2", 1);
  ros::Publisher full_pub_test_3 = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> > ("full_point_clouds_test_roughness", 1);

  ros::Publisher pub_path = n.advertise< pcl::PointCloud<pcl::PointXYZ> > ("car_pos", 1);


  ros::Rate loop_rate(LOOP_RATE);

  while (ros::ok())
  {
    if(recieved_new){
      // Set timestamps
      ros::Time timestamp = ros::Time::now();


      // Plot path
      pathPoints.push_back(pcl::PointXYZ(odo.x_pos, odo.y_pos, odo.z_pos));
      marker_pub.publish(makeLineStrip(&pathPoints, 1.0, 0.5, 0, "path"));

      pcl::PointCloud<pcl::PointXYZ>::Ptr carPoint (new pcl::PointCloud<pcl::PointXYZ>);
      carPoint->header.frame_id = "visualization_frame";
      carPoint->push_back(pcl::PointXYZ(odo.x_pos, odo.y_pos, odo.z_pos));
      pub_path.publish(carPoint);


      laserpoints->header.frame_id = "visualization_frame";
      pub.publish(laserpoints); // projected versions
      floorpoints->header.frame_id = "visualization_frame";
      pub_floor.publish(floorpoints); // projected versions

      laserpoints_colored_1->header.frame_id = "visualization_frame";
      pub_test_1.publish(laserpoints_colored_1); // projected versions
      laserpoints_colored_2->header.frame_id = "visualization_frame";
      pub_test_2.publish(laserpoints_colored_2); // projected versions
      roughness_laserpoints->header.frame_id = "visualization_frame";
      pub_test_3.publish(roughness_laserpoints); // projected versions

      marker_pub.publish(makeLineList(&(* verticalFromList), &(* verticalToList), 1.0, 0, 0, 0.01, 0.8, "vertical"));
      marker_pub.publish(makeLineList(&(* horizonalFromList), &(* horizonalToList), 0, 1.0, 0, 0.01, 0.8, "horizontal"));

      full_laserpoints->header.frame_id = "visualization_frame";
      full_pub.publish(full_laserpoints); // projected versions
      full_floorpoints->header.frame_id = "visualization_frame";
      full_pub_floor.publish(full_floorpoints); // projected versions

      full_laserpoints_colored_1->header.frame_id = "visualization_frame";
      full_pub_test_1.publish(full_laserpoints_colored_1); // projected versions
      full_laserpoints_colored_2->header.frame_id = "visualization_frame";
      full_pub_test_2.publish(full_laserpoints_colored_2); // projected versions
      full_roughness_laserpoints->header.frame_id = "visualization_frame";
      full_pub_test_3.publish(full_roughness_laserpoints); // projected versions
      marker_pub.publish(makeLineList(&(* full_verticalFromList), &(* full_verticalToList), 1.0, 0, 0, 0.01, 0.8, "full_vertical"));
      marker_pub.publish(makeLineList(&(* full_horizonalFromList), &(* full_horizonalToList), 0, 1.0, 0, 0.01, 0.8, "full_horizontal"));

      // Display time-duration of publish
      ros::Duration publishDone = ros::Time::now() - timestamp;
      //std::cout << "Duration of publish: " << publishDone << std::endl << std::endl;

      recieved_new = false;
    }else{

      pub.publish(laserpoints); // projected versions
      pub_floor.publish(floorpoints); // projected versions

      pub_test_1.publish(laserpoints_colored_1); // projected versions
      pub_test_2.publish(laserpoints_colored_2); // projected versions
      pub_test_3.publish(roughness_laserpoints); // projected versions

      marker_pub.publish(makeLineList(&(* verticalFromList), &(* verticalToList), 1.0, 0, 0, 0.01, 0.8, "vertical"));
      marker_pub.publish(makeLineList(&(* horizonalFromList), &(* horizonalToList), 0, 1.0, 0, 0.01, 0.8, "horizontal"));


      full_pub.publish(full_laserpoints); // projected versions
      full_pub_floor.publish(full_floorpoints); // projected versions
      full_pub_test_1.publish(full_laserpoints_colored_1); // projected versions
      full_pub_test_2.publish(full_laserpoints_colored_2); // projected versions
      full_pub_test_3.publish(full_roughness_laserpoints); // projected versions

      marker_pub.publish(makeLineList(&(* full_verticalFromList), &(* full_verticalToList), 1.0, 0, 0, 0.01, 0.8, "full_vertical"));
      marker_pub.publish(makeLineList(&(* full_horizonalFromList), &(* full_horizonalToList), 0, 1.0, 0, 0.01, 0.8, "full_horizontal"));

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}