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
        std::string topicsClouds;
        nh.param("transformTolerance",transformTolerance,0.2);
        nh.param("pointCloudSource",topicsClouds,std::string(""));
        ROS_INFO("Subscribed to pointCloud topics: %s",topicsClouds.c_str());



    }
}

