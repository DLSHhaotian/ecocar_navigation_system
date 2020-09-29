//
// Created by dlsh on 2020/9/19.
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <cmath>
int main(int argc,char** argv){
    ros::init(argc,argv,"tf_listener");
    ros::NodeHandle node;
    ros::service::waitForService("spawn");
    ros::ServiceClient addTurtle=node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    addTurtle.call(srv);

    ros::Publisher turtle2_vel=node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);

    tf::TransformListener lis;

    ros::Rate rate(10);
    while(node.ok()){
        tf::StampedTransform transform;
        try
        {

            lis.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            lis.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        geometry_msgs::Twist vel_send;
        vel_send.angular.z=3*atan2(transform.getOrigin().y(),transform.getOrigin().x());
        vel_send.linear.x=0.8*sqrt(pow(transform.getOrigin().x(),2)+pow(transform.getOrigin().y(),2));
        turtle2_vel.publish(vel_send);
        rate.sleep();
    }
    return 0;
}
