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
#include <message_filters/subscriber.h>
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

        void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                const boost::shared_ptr<costmap::sensorInfo_buffer>& buffer);
        void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                 const boost::shared_ptr<costmap::sensorInfo_buffer>& buffer);

    protected:
        bool getObstaclePointCloud(std::vector<costmap::sensorInfo>& pointCloud_buffers);
        virtual void rayTraceFreespace(const costmap::sensorInfo& pointCloud,double* minX,double* minY,double* maxX,double* maxY);
        void updateRayTraceBound(double ox, double oy, double wx, double wy, double range,
                                 double* minX, double* minY, double* maxX, double* maxY);
        std::string globalFrame_;
        double max_obstacleHeight_;
        std::vector<boost::shared_ptr<message_filters::SubscriberBase> > pointCloud_subscribers_;
        std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > pointCloud_notifiers_;
        std::vector<boost::shared_ptr<costmap::sensorInfo_buffer> > pointCloud_buffers_;
        bool isLocalMap;
    };
}
#endif //SRC_OBSTACLEMAPLAYER_H
