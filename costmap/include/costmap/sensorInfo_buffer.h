//
// Created by dlsh on 2020/9/23.
//

#ifndef SRC_SENSORINFO_BUFFER_H
#define SRC_SENSORINFO_BUFFER_H
#include <vector>
#include <list>
#include <string>
#include <ros/time.h>
#include <costmap/sensorInfo.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>

namespace costmap{
    class sensorInfo_buffer{
    public:
        sensorInfo_buffer(std::string topicName, double sensorInfo_keepTime, double expected_updateRate,
                          double min_obstacleHeight, double max_obstacleHeight, double obstacleRange,
                          double raytraceRange, tf2_ros::Buffer& tf2Buffer, std::string globalFrame,
                          std::string sensorFrame, double tfTolerance);
        ~sensorInfo_buffer();

        bool setGolbalFrame(const std::string new_globalFrame);
        void storeCloudIn(const sensor_msgs::PointCloud2& pointCloud);
        //get a copied vector
        void getCloud(std::vector<sensorInfo>& sensorInfoList);
        void removeOldCloud();
        void lock();
        void unlock();
        bool isCurrent();
    private:
        tf2_ros::Buffer& tf2Buffer_;
        const ros::Duration sensorInfo_keepTime_;
        const ros::Duration expected_updateRate_;
        ros::Time lastUpdated_;
        std::string globalFrame_;
        std::string sensorFrame_;
        std::list<sensorInfo> sensorInfoList_;
        std::string topicName_;
        double min_obstacleHeight_, max_obstacleHeight_;
        boost::recursive_mutex sensorInfoMutex_;
        double obstacleRange_, raytraceRange_;
        double tfTolerance_;
    };


}

#endif //SRC_SENSORINFO_BUFFER_H
