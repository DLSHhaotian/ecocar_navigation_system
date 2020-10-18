#pragma once
#include "Hull.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include "Odometry.h"


class ObstacleHull
{
  private:
    bool merged_hull_updated;
  public:
    bool visible_from_car;
    typedef std::shared_ptr<ObstacleHull> ptr;
    uint32_t id;
    uint8_t type;
    std::vector<Hull::ptr> parents;
    Hull merged_hull;
    ObstacleHull(uint32_t * id_top, Hull::ptr first_hull);
    bool isEmpty();
    void addParent(Hull::ptr new_hull);
    void remakeHull();
    void timeTrim(float seconds);
    void mergeWithObstacle(ObstacleHull::ptr other_obstacle);
    lidar_package::obst toMsg();
    lidar_package::obsts obstaclesToMsg(std::vector< ObstacleHull::ptr > * obstacles, Odometry * odo, ros::Time timestamp);
    void printObstacle();
};