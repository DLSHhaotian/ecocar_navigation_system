#ifndef OBSTACLE_TRACKER_h
#define OBSTACLE_TRACKER_h

#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_package/classes/ObstacleHull.h"
#include "lidar_package/classes/Odometry.h"

// Message types
#include "lidar_package/point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"

extern lidar_package::obsts obstaclesToMsg(std::vector< ObstacleHull::ptr > * obstacles, Odometry * odo, ros::Time timestamp);
extern void pairHullsToObstacles(std::vector< ObstacleHull::ptr > * obstacles, std::vector< Hull::ptr > * hulls, uint32_t * obstacle_id_top, std::vector<int> * color_id_vector = 0);


// Line of sight checking

class ObstaclePoint
{
  public:
    float x;
    float y;
    ObstacleHull::ptr obstacle;
    uint32_t index;
    ObstaclePoint(float x_in = 0, float y_in = 0){
      x = x_in;
      y = y_in;
    }
    ObstaclePoint(ObstacleHull::ptr obstacle_in, uint32_t index_in){
      obstacle = obstacle_in;
      index = index_in;
      pcl::PointXYZ p = obstacle_in->merged_hull.vertices->at(index_in);
      x = p.x;
      y = p.y;
    };
    // comparison is done first on y coordinate and then on x coordinate
    bool operator < (ObstaclePoint b){
        if (y != b.y) return y < b.y;
        return x < b.x;
    }
    bool operator == (ObstaclePoint b){
        return obstacle->id == b.obstacle->id && index == b.index;
    }

    uint32_t next(){
      uint32_t next_index = index + 1;
      if(obstacle->merged_hull.size() == next_index) return 0;
      return next_index;
    }
    uint32_t prev(){
      if(index == 0) return obstacle->merged_hull.size() - 1;
      return index - 1;
    }

    // returns -1 if a -> b -> c forms a clockwise turn,
    // +1 for a counter-clockwise turn, 0 if they are collinear
    int cw(ObstaclePoint a, ObstaclePoint b, ObstaclePoint c){
      float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
      if (area > 0)
          return 1;
      else if (area < 0)
          return -1;
      return 0;
    }
    // returns square of Euclidean distance between two points
    float sqrDist(ObstaclePoint b){
      float dx = x - b.x, dy = y - b.y;
      return dx * dx + dy * dy;
    }
    pcl::PointXYZ toPoint(){
      return pcl::PointXYZ(x, y, 0);
    }
    void printObstaclePoint(){
      ROS_INFO("ObstaclePoint (x: %f, y: %f) has obstacle id %d and index %d", x, y, obstacle->id, index);
    }

};

class ObstacleSegment
{
  public:
    ObstaclePoint start;
    ObstaclePoint end;
    float a, b, t;
    ObstacleSegment(ObstaclePoint start_in, ObstaclePoint end_in){
      start = start_in;
      end = end_in;
      a = start.y - end.y;
      b = end.x - start.x;
      t = start.x*end.y - end.x*start.y;
    }
    float distanceToCrossingSqr(ObstacleSegment * other){
      // Cross
      float a_temp = b*other->t - other->b*t;
      float b_temp = other->a*t - a*other->t;
      float t_temp = a*other->b - other->a*b;
      // Reduce to unity if not parallel
      if(t_temp == 0) return -1;
      a_temp /= t_temp;
      b_temp /= t_temp;
      // Return distance
      float x_dist = (a_temp-start.x);
      float y_dist = (b_temp-start.y);
      return x_dist*x_dist + y_dist*y_dist;    
    }
    pcl::PointXYZ crossPoint(ObstacleSegment * other){
      // Cross
      float a_temp = b*other->t - other->b*t;
      float b_temp = other->a*t - a*other->t;
      float t_temp = a*other->b - other->a*b;
      // Reduce to unity if not parallel
      if(t_temp == 0)  return pcl::PointXYZ(0, 0, 1);
      a_temp /= t_temp;
      b_temp /= t_temp;
      return pcl::PointXYZ(a_temp, b_temp, 0);    
    }
    void printObstacleSegment(){
      ROS_INFO("ObstacleSegment goes from (%f, %f) to (%f, %f) and has obstacle id %d", start.x, start.y, end.x, end.y, start.obstacle->id);
    }
};

ObstaclePoint pivot = ObstaclePoint(0, 0); //Inital values
ObstacleSegment pivotSegment = ObstacleSegment(pivot, pivot); //Inital values
void setPivot(ObstaclePoint p){
  pivot = p;
}
void updatePivotSegment(ObstaclePoint p){
  pivotSegment = ObstacleSegment(pivot, p);
}

// used for sorting points according to polar order w.r.t the pivot
bool POLAR_ORDER(ObstaclePoint a, ObstaclePoint b) {
  int order = pivot.cw(pivot, a, b);
  if (order == 0)
      return pivot.sqrDist(a) < pivot.sqrDist(b);
  return (order == -1);
}

void addNextObstacleSegmentsFromPoint(ObstaclePoint * p, std::vector<ObstacleSegment> * segments){
  uint32_t next = p->next();
  uint32_t prev = p->prev();
  if(next == prev){
      ObstaclePoint p_next = ObstaclePoint(p->obstacle,  next);
      if(POLAR_ORDER(* p, p_next))
        segments->push_back(ObstacleSegment(* p, p_next));
  }else{
      ObstaclePoint p_next = ObstaclePoint(p->obstacle,  next);
      if(POLAR_ORDER(* p, p_next))
        segments->push_back(ObstacleSegment(* p, ObstaclePoint(p->obstacle,  next)));

      ObstaclePoint p_prev = ObstaclePoint(p->obstacle,  prev);
      if(POLAR_ORDER(* p, p_prev))
        segments->push_back(ObstacleSegment(* p, p_prev));
  }
}

void sortObstaclePointsByAngle(std::vector<ObstaclePoint> * obstacle_points){
  std::sort(obstacle_points->begin(), obstacle_points->end(), POLAR_ORDER);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr exportObstaclePoints(std::vector<ObstaclePoint> obstacle_points){
  pcl::PointCloud<pcl::PointXYZ>::Ptr exportPoints (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < obstacle_points.size(); i++)
  {
    exportPoints->push_back(pcl::PointXYZ(obstacle_points.at(i).x,obstacle_points.at(i).y, 0));
  }
  return exportPoints;
}

int findClosestObstaclePoint(std::vector<ObstaclePoint> * obstacle_points){
  if(obstacle_points->empty()){
    ROS_ERROR("Got empty vector for closest point search!");
    return -1;
  }
  float best_distance_sqr = pivot.sqrDist(obstacle_points->at(0));
  int best_index = 0; 
  for (size_t i = 1; i < obstacle_points->size(); i++)
  {
    float distance_sqr = pivot.sqrDist(obstacle_points->at(i));
    if(distance_sqr < best_distance_sqr){
      best_index = i;
      best_distance_sqr = distance_sqr;
    }
  }
  return best_index;
}

std::vector<ObstaclePoint> makeObstaclePoints(std::vector< ObstacleHull::ptr > * src){
  // TODO: ALSO CLEAR TYPE, this should be in its own function.
  // Set size of vector and clear types
  uint32_t total_size = 0;
  std::vector<ObstaclePoint> obstacle_points;
  for (uint32_t i = 0; i < src->size(); i++)
  {
    total_size +=  src->at(i)->merged_hull.size();
    src->at(i)->type = 0;
  }
  obstacle_points.resize(total_size);

  // Fill vector
  int index = 0;
  for (uint32_t i = 0; i < src->size(); i++)
  {
    for (uint32_t j = 0; j < src->at(i)->merged_hull.size(); j++){
      obstacle_points.at(index) = ObstaclePoint(src->at(i), j);
      index++;
    }
  }
  return obstacle_points;
}

#endif