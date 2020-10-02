//
// Created by dlsh on 2020/9/20.
//

#include <ros/ros.h>
#include <costmap/costmap_system.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_node");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    costmap::costmap_system lcr("costmap", buffer);

    ros::spin();

    return (0);
}
