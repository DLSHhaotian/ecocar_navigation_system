//
// Created by dlsh on 2020/9/22.
//

#include <costmap/obstacleMapLayer.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <costmap/cost_value.h>
using costmap::NO_INFORMATION;
using costmap::LETHAL_OBSTACLE;
using costmap::FREE_SPACE;
namespace costmap{
    obstacleMapLayer::obstacleMapLayer() {}
    obstacleMapLayer::~obstacleMapLayer() {}

    void obstacleMapLayer::ownInitialize() {
        ros::NodeHandle nh("~/"+name_), g_nh;
        isLocalMap=manager_->isLocalMap();

        costDefaultValue_=NO_INFORMATION;
        //match the parent map's size(master map)
        obstacleMapLayer::matchParentSize();
        current_=true;

        globalFrame_=manager_->getGlobalFrame();
        double transformTolerance;
        std::string topicClouds;
        nh.param("transform_tolerance_",transformTolerance,0.2);
        nh.param("pointCloudSource",topicClouds,std::string(""));
        ROS_INFO("Subscribed to pointCloud topics: %s",topicClouds.c_str());

        ros::NodeHandle sourceTopic(nh,topicClouds);

        double sensorInfo_keepTime=0, expected_UpdateRate=0,min_obstacleHeight=0,max_obstacleHeight=0;
        std::string topic,sensorFrame,dataType;

        sourceTopic.param("topic", topic, topicClouds);
        sourceTopic.param("sensorFrame", sensorFrame, std::string(""));
        sourceTopic.param("sensorInfo_keepTime", sensorInfo_keepTime, 0.0);
        sourceTopic.param("expected_UpdateRate", expected_UpdateRate, 0.0);
        sourceTopic.param("dataType", dataType, std::string("PointCloud"));
        sourceTopic.param("min_obstacleHeight", min_obstacleHeight, 0.0);
        sourceTopic.param("max_obstacleHeight", max_obstacleHeight, 2.0);

        std::string raytrace_range_param_name, obstacle_range_param_name;

        // get the obstacle range for the sensor
        double obstacleRange = 2.5;
        if (sourceTopic.searchParam("obstacle_range", obstacle_range_param_name))
        {
            sourceTopic.getParam(obstacle_range_param_name, obstacleRange);
        }

        // get the raytrace range for the sensor
        double raytraceRange = 3.0;
        if (sourceTopic.searchParam("raytrace_range", raytrace_range_param_name))
        {
            sourceTopic.getParam(raytrace_range_param_name, raytraceRange);
        }
        ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", topicClouds.c_str(), topic.c_str(),
                  sensorFrame.c_str());
        //put into the pointClound_buffer
        pointCloud_buffers_.push_back(boost::shared_ptr<sensorInfo_buffer> (new sensorInfo_buffer(topic,sensorInfo_keepTime,expected_UpdateRate,min_obstacleHeight,max_obstacleHeight,obstacleRange,raytraceRange,*tf_,globalFrame_,sensorFrame,transformTolerance)));

        ROS_DEBUG(
                "Created an observation buffer for source %s, topic %s, global frame: %s, "
                "expected update rate: %.2f, observation persistence: %.2f",
                topicClouds.c_str(), topic.c_str(), globalFrame_.c_str(),expected_UpdateRate, sensorInfo_keepTime);

        if(dataType=="PointCloud"){
            //message_filter::subscriber
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh,topic,50));
            //tf2_ros::MessageFilter
            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud>> filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub,*tf_,globalFrame_,50,g_nh));
            //callback register
            filter->registerCallback(boost::bind(&obstacleMapLayer::pointCloudCallback,this,_1,pointCloud_buffers_.back()));

            pointCloud_subscribers_.push_back(sub);
            pointCloud_notifiers_.push_back(filter);
        }
        else {
            //message_filter::subscriber
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh,topic,50));
            //tf2_ros::MessageFilter
            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub,*tf_,globalFrame_,50,g_nh));
            //callback register
            filter->registerCallback(boost::bind(&obstacleMapLayer::pointCloud2Callback,this,_1,pointCloud_buffers_.back()));

            pointCloud_subscribers_.push_back(sub);
            pointCloud_notifiers_.push_back(filter);
        }
    }
    void obstacleMapLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr &message,
                                              const boost::shared_ptr<costmap::sensorInfo_buffer> &buffer) {
        sensor_msgs::PointCloud2 pointCloud2;
        if(!sensor_msgs::convertPointCloudToPointCloud2(*message,pointCloud2)){
            ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
            return;
        }
        buffer->lock();
        buffer->storeCloudIn(pointCloud2);
        buffer->unlock();
    }
    void obstacleMapLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &message,
                                               const boost::shared_ptr<costmap::sensorInfo_buffer> &buffer) {
        buffer->lock();
        buffer->storeCloudIn(*message);
        buffer->unlock();
    }
    bool obstacleMapLayer::getObstaclePointCloud(std::vector<costmap::sensorInfo> &pointCloud_buffers) {
        bool current=true;
        for(unsigned int i=0;i<pointCloud_buffers_.size();++i){
            pointCloud_buffers_[i]->lock();
            pointCloud_buffers_[i]->getCloud(pointCloud_buffers);
            current=pointCloud_buffers_[i]->isCurrent()&&current;
            pointCloud_buffers_[i]->unlock();
        }
    }
    void obstacleMapLayer::rayTraceFreespace(const costmap::sensorInfo &pointCloud, double *minX, double *minY,
                                             double *maxX, double *maxY) {
        double originSensorX=pointCloud.origin_.x;//geometry_msgs::Point
        double originSensorY=pointCloud.origin_.y;
        const sensor_msgs::PointCloud2& pointCloud2=*(pointCloud.pointCloud_);
        //convert the origin of sensor from world to map
        unsigned int sensorX0=0,sensorY0=0;
        if(!worldToMap(originSensorX,originSensorY,sensorX0,sensorY0)){
            ROS_WARN_THROTTLE(
                    1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
                    originSensorX, originSensorY);
            return;
        }
        //update bounds for the origin of sensor in world coordinate
        updateBoundsForPoint(originSensorX,originSensorY,minX,minY,maxX,maxY);
        //For the point cloud of each obstacle, the straight line to the point cloud will be set as feasible
        sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud2,"x");
        sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud2,"y");

        //compute the Xn,Yn of costmap related to the costmap origin
        double originMapX=originX_;
        double originMapY=originY_;
        double endMapX=originX_+sizeX_*resolution_;
        double endMapY=originY_+sizeY_*resolution_;

        for(;iterX!=iterX.end();++iterX,++iterY){
            double endSensorX=*iterX;
            double endSensorY=*iterY;
            //make sure that the point is within the costmap, and if not, scale it
            double subSensorX=endSensorX-originSensorX;
            double subSensorY=endSensorY-originSensorY;

            //compare with the costmap's origin and end
            //similar triangle
            if(endSensorX<originMapX){
                double scale=(originMapX-originSensorX)/subSensorX;
                endSensorX=originMapX;
                endSensorY=originSensorY+scale*subSensorY;
            }
            if(endSensorY<originMapY){
                double scale=(originMapY-originSensorY)/subSensorY;
                endSensorX=originMapX+scale*subSensorX;
                endSensorY=originSensorY;
            }
            if(endSensorX>endMapX){
                double scale=(endMapX-originSensorX)/subSensorX;
                endSensorX=endMapX-0.01;
                endSensorY=originSensorY+scale*subSensorY;
            }
            if(endSensorY>endMapY){
                double scale=(endMapY-originSensorY)/subSensorY;
                endSensorX=originSensorX+scale*subSensorX;
                endSensorY=endMapY-0.01;
            }

            //convert the processed endPoint(scaled in costmap) from world to map
            unsigned sensorXn=0,sensorYn=0;
            if(!worldToMap(endSensorX,endSensorY,sensorXn,sensorYn))
                continue;
            //convert the givin rayTraceRange from world to map
            unsigned int map_rayTraceRange=worldToMapDistance(pointCloud.raytraceRange_);
            markCell marker(mapChar_,FREE_SPACE);
            //clear the obstacle
            rayTraceLine(marker,sensorX0,sensorY0,sensorXn,sensorYn);
            //update bounds
            updateRayTraceBound(originSensorX,originMapY,endSensorX,endSensorY,pointCloud.raytraceRange_,minX,minY,maxX,maxY);

        }
    }
    void obstacleMapLayer::updateRayTraceBound(double ox, double oy, double wx, double wy, double range, double *minX,
                                               double *minY, double *maxX, double *maxY) {
        double dx=wx-ox;
        double dy=wy-oy;
        double dist=std::hypot(dx,dy);
        double scale=std::min(1.0,range/dist);
        double ex=ox+dx*scale;
        double ey=oy+dy*scale;

        updateBoundsForPoint(ex,ey,minX,minY,maxX,maxY);
    }
    void obstacleMapLayer::updateBound(double robotX, double robotY, double robotTheta, double *minX, double *minY,
                                       double *maxX, double *maxY) {
        if(isLocalMap)
            updateOrigin(robotX-get_mapSizeX_meter()/2,robotY-get_mapSizeY_meter()/2);
        bool current=true;
        //get the obstacle's point cloud
        std::vector<sensorInfo> pointCloudInfoList;
        current=current && getObstaclePointCloud(pointCloudInfoList);
        current_=current;
        //raytrace free space
        for(unsigned int i=0;i<pointCloudInfoList.size();++i){
            rayTraceFreespace(pointCloudInfoList[i],minX,minY,maxX,maxY);
        }
        //marking obstacle
        for(std::vector<sensorInfo>::const_iterator it=pointCloudInfoList.begin();it!=pointCloudInfoList.end();++it){
            const sensorInfo& pointCloudInfo=*it;
            const sensor_msgs::PointCloud2& pointCloud=*(pointCloudInfo.pointCloud_);

            double obstacleRange_square=pointCloudInfo.obstacleRange_*pointCloudInfo.obstacleRange_;

            sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud,"x");
            sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud,"y");
            sensor_msgs::PointCloud2ConstIterator<float> iterZ(pointCloud,"z");

            //for every point
            for(;iterX!=iterX.end();++iterX,++iterY,++iterZ){
                double obstacleX=*iterX;
                double obstacleY=*iterY;
                double obstacleZ=*iterZ;

                //throw the obstacle which is too high
                if(obstacleZ>max_obstacleHeight_){
                    ROS_DEBUG("The point is too high");
                    continue;
                }
                //throw the obstacle which is too far
                double dist_square=std::pow((obstacleX-pointCloudInfo.origin_.x),2)+std::pow((obstacleY-pointCloudInfo.origin_.y),2)+std::pow((obstacleZ-pointCloudInfo.origin_.z),2);
                if(dist_square>=obstacleRange_square){
                    ROS_DEBUG("The point is too far away");
                    continue;
                }
                //compute the map coordinate of obstacle
                unsigned obstacleMapX=0,obstacleMapY=0;
                if(!worldToMap(obstacleX,obstacleY,obstacleMapX,obstacleMapY)){
                    ROS_DEBUG("Computing map coordinates failed");
                    continue;
                }

                unsigned int obstacle_index=get_MapToIndex(obstacleMapX,obstacleMapY);
                mapChar_[obstacle_index]=LETHAL_OBSTACLE;
                updateBoundsForPoint(obstacleX,obstacleY,minX,minY,maxX,maxY);
            }
        }
    }
    void obstacleMapLayer::updateCost(costmap_base &master_grid, int boundX0, int boundY0, int boundXn, int boundYn) {
        updateWithOverWrite(master_grid,boundX0,boundY0,boundY0,boundYn);
    }
}

