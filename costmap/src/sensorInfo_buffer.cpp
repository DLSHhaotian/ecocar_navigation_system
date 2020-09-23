//
// Created by dlsh on 2020/9/23.
//

#include "costmap/sensorInfo_buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace costmap{
    sensorInfo_buffer::sensorInfo_buffer(std::string topicName, double sensorInfo_keepTime, double expected_updateRate,
                                         double min_obstacleHeight, double max_obstacleHeight, double obstacleRange,
                                         double raytraceRange, tf2_ros::Buffer &tf2Buffer, std::string globalFrame,
                                         std::string sensorFrame, double tfTolerance) : tf2Buffer_(tf2Buffer), sensorInfo_keepTime_(sensorInfo_keepTime), expected_updateRate_(expected_updateRate),
                                                                                        lastUpdated_(ros::Time::now()), globalFrame_(globalFrame), sensorFrame_(sensorFrame), topicName_(topicName),
                                                                                        min_obstacleHeight_(min_obstacleHeight), max_obstacleHeight_(max_obstacleHeight),obstacleRange_(obstacleRange), raytraceRange_(raytraceRange), tfTolerance_(tfTolerance){}

    sensorInfo_buffer::~sensorInfo_buffer() {}
    bool sensorInfo_buffer::setGolbalFrame(const std::string new_globalFrame) {
        ros::Time transform_time = ros::Time::now();
        std::string tf_error;

        geometry_msgs::TransformStamped transformStamped;
        if (!tf2Buffer_.canTransform(new_globalFrame, globalFrame_, transform_time, ros::Duration(tfTolerance_), &tf_error))
        {
            ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_globalFrame.c_str(),
                      globalFrame_.c_str(), tfTolerance_, tf_error.c_str());
            return false;
        }
        std::list<sensorInfo>::iterator listIt;
        for(listIt=sensorInfoList_.begin();listIt!=sensorInfoList_.end();++listIt){
            try{
                sensorInfo& tempInfo=*listIt;
                geometry_msgs::PointStamped origin;
                origin.header.frame_id=globalFrame_;
                origin.header.stamp=transform_time;
                origin.point=tempInfo.origin_;

                //Frist transform the origin point to new global frame
                tf2Buffer_.transform(origin,origin,new_globalFrame);
                tempInfo.origin_=origin.point;
                //Then transform the pointcloud to new global frame
                tf2Buffer_.transform(*(tempInfo.pointCloud_),*(tempInfo.pointCloud_),new_globalFrame);

            }
            catch(tf2::TransformException& ex){
                ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", globalFrame_.c_str(),
                          new_globalFrame.c_str(), ex.what());
                return false;
            }
        }
        globalFrame_=new_globalFrame;
        return true;
    }
    void sensorInfo_buffer::storeCloudIn(const sensor_msgs::PointCloud2 &pointCloud) {
        sensorInfoList_.push_front(sensorInfo());

        std::string originFrame= sensorFrame_==""? pointCloud.header.frame_id : sensorFrame_;

        geometry_msgs::PointStamped originGlobal;

        try{
            geometry_msgs::PointStamped originLocal;
            originLocal.header.stamp=pointCloud.header.stamp;
            originLocal.header.frame_id=originFrame;
            originLocal.point.x=0;
            originLocal.point.y=0;
            originLocal.point.z=0;
            //compute the origin point in global frame
            tf2Buffer_.transform(originLocal,originGlobal,globalFrame_);
            //convert type from geometry_msgs::PointStamped to geometry_msgs::Point origin_
            tf2::convert(originGlobal.point,sensorInfoList_.front().origin_);

            sensorInfoList_.front().raytraceRange_=raytraceRange_;
            sensorInfoList_.front().obstacleRange_-obstacleRange_;

            sensor_msgs::PointCloud2 pointCloudGlobal;
            //transform the pointclound to global frame
            tf2Buffer_.transform(pointCloud,pointCloudGlobal,globalFrame_);
            //put computed pointCloudGlobal in our list
            sensor_msgs::PointCloud2 pointCloundNew=*(sensorInfoList_.front().pointCloud_);
            pointCloundNew.height=pointCloudGlobal.height;
            pointCloundNew.width=pointCloudGlobal.width;
            pointCloundNew.fields=pointCloudGlobal.fields;
            pointCloundNew.is_bigendian=pointCloudGlobal.is_bigendian;
            pointCloundNew.point_step=pointCloudGlobal.point_step;
            pointCloundNew.row_step=pointCloudGlobal.row_step;
            pointCloundNew.is_dense=pointCloudGlobal.is_dense;

            unsigned int pointCloudSize=pointCloundNew.height*pointCloundNew.width;
            //use modifier
            sensor_msgs::PointCloud2Modifier modifier(pointCloundNew);
            //resize the pointcloud
            modifier.resize(pointCloudSize);

            //put the data of pointCloudGlobal in our list
            //iterator of pointCloudGlobal of z coordinate
            sensor_msgs::PointCloud2Iterator<float> iterZ(pointCloudGlobal,"z");
            //iterator of data of pointCLoudGlobal
            std::vector<unsigned char>::const_iterator iterGlobal=pointCloudGlobal.data.begin();
            //iterator of data of pointCLoudNew
            std::vector<unsigned char>::iterator iterNew=pointCloundNew.data.begin();

            //suited point number
            unsigned int pointCloudNewSize=0;
            for(;iterGlobal!=pointCloudGlobal.data.end();++iterZ,iterGlobal+=pointCloudGlobal.point_step){
                if((*iterZ)<=max_obstacleHeight_&&(*iterZ)>=min_obstacleHeight_){
                    std::copy(iterGlobal,iterGlobal+pointCloudGlobal.point_step,iterNew);
                    iterNew+=pointCloudGlobal.point_step;
                    ++pointCloudNewSize;
                }
            }
            //resize the suited data
            modifier.resize(pointCloudNewSize);
            pointCloundNew.header.stamp=pointCloud.header.stamp;
            pointCloundNew.header.frame_id=pointCloudGlobal.header.frame_id;
        }
        catch (tf2::TransformException& ex)
        {
            // if an exception occurs, we need to remove the empty observation from the list
            sensorInfoList_.pop_front();
            ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensorFrame_.c_str(),
                      pointCloud.header.frame_id.c_str(), ex.what());
            return;
        }
        lastUpdated_=ros::Time::now();
        //remove the out-dated data
        removeOldCloud();
    }
    void sensorInfo_buffer::removeOldCloud() {
        if(!sensorInfoList_.empty()){
            std::list<sensorInfo>::iterator iterSensorInfo=sensorInfoList_.begin();
            if(sensorInfo_keepTime_==ros::Duration(0)){
                sensorInfoList_.erase(++iterSensorInfo,sensorInfoList_.end());
                return;
            }
            // check if the observation is out of date
            for(iterSensorInfo=sensorInfoList_.begin();iterSensorInfo!=sensorInfoList_.end();++iterSensorInfo){
                sensorInfo& sensorInfoObj=*iterSensorInfo;
                if((lastUpdated_-sensorInfoObj.pointCloud_->header.stamp)>sensorInfo_keepTime_){
                    sensorInfoList_.erase(iterSensorInfo,sensorInfoList_.end());
                    return;
                }
            }
        }
    }
    void sensorInfo_buffer::getCloud(std::vector<sensorInfo> &sensorInfoList) {
        removeOldCloud();
        std::list<sensorInfo>::iterator iterSensorInfo;
        for(iterSensorInfo=sensorInfoList_.begin();iterSensorInfo!=sensorInfoList_.end();++iterSensorInfo){
            sensorInfoList.push_back(*iterSensorInfo);
        }
    }

    void sensorInfo_buffer::lock() {
        sensorInfoMutex_.lock();
    }
    void sensorInfo_buffer::unlock() {
        sensorInfoMutex_.unlock();
    }
}