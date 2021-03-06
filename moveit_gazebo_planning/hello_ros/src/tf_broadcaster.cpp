//
// Created by dlsh on 2020/9/19.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <boost/bind.hpp>
#include <cstring>

void poseCallback(const turtlesim::Pose::ConstPtr& msg,std::string turtle_name){
    //f broadcaster
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));
    tf::Quaternion q;
    q.setRPY(0,0,msg->theta);
    transform.setRotation(q);
    //publish the tf
    ROS_INFO("start broadcast tf");
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));

}

int main(int argc,char** argv){
    ros::init(argc,argv,"tf_broadcaster");
    if(argc!=2){
        ROS_ERROR("need turtle name as argument");
        return -1;
    }
    std::string turtle_name=argv[1];

    ros::NodeHandle node;
    ros::Subscriber subPose=node.subscribe<turtlesim::Pose>(turtle_name+"/pose",10,boost::bind(poseCallback,_1,turtle_name));
    ros::spin();
    return 0;
}

