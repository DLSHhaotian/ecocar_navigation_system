//
// Created by dlsh on 2020/9/22.
//

#ifndef SRC_STATICMAPLAYER_H
#define SRC_STATICMAPLAYER_H
#include <ros/ros.h>
#include <costmap/cost_value.h>
#include <costmap/costmapLayer.h>
//#include <costmap/costLayer_manager.h>
#include <nav_msgs/OccupancyGrid.h>
//#include <map_msgs/OccupancyGridUpdate.h>
//#include <message_filters/subscriber.h>

namespace costmap{
class staticMapLayer: public costmap::costmapLayer{
    public:
        staticMapLayer();
        virtual ~staticMapLayer();
        virtual void ownInitialize();
        virtual void activate();
        virtual void deactivate();
        virtual void reset();
        virtual void updateBound(double robotX, double robotY, double robotTheta, double* minX, double* minY,
                                 double* maxX, double* maxY);
        virtual void updateCost(costmap_base& master_grid, int boundX0, int boundY0, int boundXn, int boundYn);
        virtual void matchParentSize();
    private:
        void inputMap(const nav_msgs::OccupancyGridConstPtr& new_map);
        unsigned char interpretValue(unsigned char value);

        std::string globalFrame_;
        std::string mapFrame_;
        bool mapReceived_;
        bool dataUpdated_;
        unsigned int staticX_, staticY_, staticSizeX_, staticSizeY_;
        bool first_map_only_;
        ros::Subscriber map_sub_;
    };
}



#endif //SRC_STATICMAPLAYER_H
