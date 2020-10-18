#include "ObstacleHull.h"

ObstacleHull::ObstacleHull(uint32_t * id_top, Hull::ptr first_hull){
  * id_top = * id_top + 1;
  id = * id_top;
  type = 0;
  parents.push_back(first_hull);
  merged_hull = first_hull->copyHull();
  merged_hull_updated = true;
  visible_from_car = false;
};

bool ObstacleHull::isEmpty(){
  if(parents.size() == 0) return true;
  return false;
};

void ObstacleHull::addParent(Hull::ptr new_hull){
  parents.push_back(new_hull);
  merged_hull_updated = false;
};

void ObstacleHull::remakeHull(){
  type = 0;
  if(!merged_hull_updated){
    merged_hull.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_parant_points (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < parents.size(); i++)
    {
      * all_parant_points += * parents.at(i)->vertices;
    }
    merged_hull = Hull(all_parant_points, true);
    merged_hull_updated = true;
  }
};

void ObstacleHull::timeTrim(float seconds){
  std::vector<Hull::ptr> new_parents;
  ros::Duration duration(seconds);
  ros::Time now = ros::Time::now();
  bool no_changes = true;
  for (size_t i = 0; i < parents.size(); i++)
  {
    if(now - parents.at(i)->timestamp < duration){
      new_parents.push_back(parents.at(i));
    }else{
      no_changes = false;
    }
  }
  parents = new_parents;
  merged_hull_updated = no_changes;
};

void ObstacleHull::mergeWithObstacle(ObstacleHull::ptr other_obstacle){
  // Merge ids by lowest/youngest
  if(id > other_obstacle->id){
    id = other_obstacle->id;
  }
  // Merge parents
  for (size_t parent_index = 0; parent_index < other_obstacle->parents.size(); parent_index++)
  {
    parents.push_back(other_obstacle->parents.at(parent_index));
  }
  // Indicate merged hull needs update
  merged_hull_updated = false;
};

lidar_package::obst ObstacleHull::toMsg(){
  lidar_package::obst obstacle_msg;
  obstacle_msg.id = id;
  obstacle_msg.parents = parents.size();
  obstacle_msg.type = type;
  obstacle_msg.vector_len = merged_hull.size();
  for (size_t j = 0; j < obstacle_msg.vector_len; j++){
    lidar_package::point point;
    point.x = merged_hull.vertices->at(j).x;
    point.y = merged_hull.vertices->at(j).y;
    point.z = merged_hull.vertices->at(j).z;
    obstacle_msg.vector_point.push_back(point);
  }
  return obstacle_msg;
};

lidar_package::obsts obstaclesToMsg(std::vector< ObstacleHull::ptr > * obstacles, Odometry * odo, ros::Time timestamp){
  lidar_package::obsts obstacles_msg;
  // int count = 0;
  odo_data odo_struct = odo->getInterpolatedData(timestamp);
  obstacles_msg.x = odo_struct.x_pos;
  obstacles_msg.y = odo_struct.y_pos;
  obstacles_msg.z = odo_struct.z_pos;
  obstacles_msg.orientation = odo_struct.orientation;
  obstacles_msg.vector_len = obstacles->size();
  obstacles_msg.timestamp = timestamp;
  for (size_t i = 0; i < obstacles_msg.vector_len; i++){
    obstacles_msg.vector_obst.push_back(obstacles->at(i)->toMsg());
    // count += obstacles->at(i)->merged_hull.size();
  }
  // ROS_INFO("Number og points on all: %d", count);
  return obstacles_msg;
};

void ObstacleHull::printObstacle(){
  ROS_INFO("Obstacle %d contains %d vertices from %d hulls", id, (int)merged_hull.size(), (int)parents.size());
};
