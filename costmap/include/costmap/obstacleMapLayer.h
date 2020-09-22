//
// Created by dlsh on 2020/9/22.
//

#ifndef SRC_OBSTACLEMAPLAYER_H
#define SRC_OBSTACLEMAPLAYER_H
#include <ros/ros.h>
#include <costmap/costmapLayer.h>
namespace costmap {
    class obstacleMapLayer : public costmap::costmapLayer {
        obstacleMapLayer();
        ~obstacleMapLayer();
    };
}
#endif //SRC_OBSTACLEMAPLAYER_H
