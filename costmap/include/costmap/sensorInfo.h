//
// Created by dlsh on 2020/9/23.
//

#ifndef SRC_SENSORINFO_H
#define SRC_SENSORINFO_H
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

namespace costmap{
    class sensorInfo{
    public:
        //default constructor
        sensorInfo():pointCloud_(new sensor_msgs::PointCloud2()),obstacleRange_(0),raytraceRange_(0){
        }

        virtual ~sensorInfo(){
            delete pointCloud_;
        }
        //constructor by giving the sensor origin and associated pointclouds
        sensorInfo(geometry_msgs::Point& origin,const sensor_msgs::PointCloud2 &pointCloud,double obstacleRange,double raytraceRange):
        origin_(origin),pointCloud_(new sensor_msgs::PointCloud2(pointCloud)),obstacleRange_(obstacleRange),raytraceRange_(raytraceRange){}

        //copy constructor
        sensorInfo(const sensorInfo& info):origin_(info.origin_),pointCloud_(new sensor_msgs::PointCloud2(*(info.pointCloud_))),obstacleRange_(info.obstacleRange_),raytraceRange_(info.raytraceRange_){}
        //constructor by giving the pointclouds only
        sensorInfo(const sensor_msgs::PointCloud2 &pointCloud,double obstacleRange):pointCloud_(new sensor_msgs::PointCloud2(pointCloud)),obstacleRange_(obstacleRange){}
        geometry_msgs::Point origin_;//sensor origin
        sensor_msgs::PointCloud2* pointCloud_;
        double obstacleRange_;
        double raytraceRange_;

    };
}
#endif //SRC_SENSORINFO_H
