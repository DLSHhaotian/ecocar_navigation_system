#include "obstacle_tracker.h"

void pairHullsToObstacles(std::vector< ObstacleHull::ptr > * obstacles, std::vector< Hull::ptr > * hulls, uint32_t * obstacle_id_top, std::vector<int> * color_id_vector){
  size_t color_list_top = 0; 
  size_t initial_obstacles_size = obstacles->size(); // Because no new hulls should fit each other
  for (size_t hull_index = 0; hull_index < hulls->size(); hull_index++)
  {
    if(hulls->at(hull_index)->size() == 0) continue; // Avoid zero hulls
    int first_obstacle_fit = -1;
    for (size_t obstacle_index = 0; obstacle_index < initial_obstacles_size; obstacle_index++)
    {
      // Checking new hull
      if(obstacles->at(obstacle_index)->merged_hull.compareToHull(hulls->at(hull_index)))
      {
        // Fits
        if(first_obstacle_fit < 0){
            // Adding Hull to obstacle
            first_obstacle_fit = obstacle_index;
            obstacles->at(first_obstacle_fit)->addParent(hulls->at(hull_index));
        }else{
            // Mergeing obstacles
            obstacles->at(first_obstacle_fit)->mergeWithObstacle(obstacles->at(obstacle_index));
            obstacles->erase(obstacles->begin() + obstacle_index);
            obstacle_index--;         // Compensate index to avoid overflow
            initial_obstacles_size--; // Compensate size to avoid overflow
        }
      } else{
        // Doesn't fit
      }
    }
    if(first_obstacle_fit < 0){
      // Adding Hull as new obstacle
      obstacles->push_back(ObstacleHull::ptr (new ObstacleHull(obstacle_id_top, hulls->at(hull_index))));
    }
    // Assign IDs to color vector
    if(color_id_vector){
      if(color_list_top >= color_id_vector->size()){
        ROS_WARN("color_list overflowing at hull %d!!!", (int) hull_index);
        color_list_top = 0;
      }
      // Avoid coloring outliers (and avoid overflow)
      while(color_id_vector->at(color_list_top) < 0 && color_list_top < color_id_vector->size()) color_list_top++; 
      // If hull is added to new obstacle
      if(first_obstacle_fit < 0){
        color_id_vector->at(color_list_top) = obstacles->back()->id;
      } else {// If hull is added to old obstacle
        color_id_vector->at(color_list_top) = obstacles->at(first_obstacle_fit)->id;
      }
      color_list_top++;
    }
  }
  
};
