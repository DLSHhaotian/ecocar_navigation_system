//
// Created by dlsh on 2020/9/22.
//

#ifndef SRC_OBSTACLEMAPLAYER_H
#define SRC_OBSTACLEMAPLAYER_H
#include <ros/ros.h>
#include <costmap/costmapLayer.h>
#include <costmap/costLayer_manager.h>
#include <costmap/sensorInfo_buffer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
namespace costmap {
    class obstacleMapLayer : public costmap::costmapLayer {
        obstacleMapLayer();
        ~obstacleMapLayer();
        virtual void ownInitialize();
        virtual void activate();
        virtual void deactivate();
        virtual void reset();
        virtual void updateBound(double robotX, double robotY, double robotTheta, double* minX, double* minY,
                                 double* maxX, double* maxY);
        virtual void updateCost(costmap_base& master_grid, int boundX0, int boundY0, int boundXn, int boundYn);

        void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                 const boost::shared_ptr<costmap::sensorInfo_buffer>& buffer);

    protected:
        std::string globalFrame_;
        double max_obstacleHeight_;
        std::vector<boost::shared_ptr<costmap::sensorInfo_buffer> > observation_buffers_;
        std::vector<boost::shared_ptr<costmap::sensorInfo_buffer> > marking_buffers_;
        std::vector<boost::shared_ptr<costmap::sensorInfo_buffer> > clearing_buffers_;
        bool isLocalMap;
    };
}
#endif //SRC_OBSTACLEMAPLAYER_H
