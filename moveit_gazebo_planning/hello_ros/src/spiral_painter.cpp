//
// Created by dlsh on 2020/9/15.
//
#include <ros/ros.h>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include "turtlesim/SetPen.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Pose.h"
//线速度增加系数
int k_linear=0;
//视窗边缘坐标，中心点坐标
const double map_min=0.1,map_max=10.9,start_x=5.5,start_y=5.5,start_theta=0;
//为了在回调函数中发送服务请求，定义为全局变量
ros::ServiceClient MoveToCenter;
turtlesim::TeleportAbsolute startPoint;
//订阅坐标后的回调函数，检查是否撞墙
void getPoseCallback(const turtlesim::Pose::ConstPtr& msg){

    if(msg->x<map_min||msg->x>map_max||msg->y<map_min||msg->y>map_max){
        ROS_INFO("Warning: hit the wall");
        startPoint.request.x=start_x;
        startPoint.request.y=start_y;
        startPoint.request.theta=start_theta;
        MoveToCenter.call(startPoint);
        k_linear=0;
    }
}
int main(int argc, char** argv)
{

    ros::init(argc, argv, "spiral_painter");
    ros::NodeHandle node;
    if(argc!=4){
        ROS_ERROR("need [r,g,b] value as argument");
        return -1;
    }
//初始化轨迹颜色参数
    std::string color_r=argv[1];
    std::string color_g=argv[2];
    std::string color_b=argv[3];
    node.setParam("/line_color/line_r",stoi(color_r));
    node.setParam("/line_color/line_g",stoi(color_g));
    node.setParam("/line_color/line_b",stoi(color_b));
//初始化服务，发布，订阅
    ros::Publisher speedPub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Subscriber getPose=node.subscribe("/turtle1/pose",10,getPoseCallback);
    ros::service::waitForService("/turtle1/set_pen");
    ros::ServiceClient lineColorChange = node.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    turtlesim::SetPen color_init;
    ros::service::waitForService("/turtle1/teleport_absolute");
    MoveToCenter = node.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    ros::Rate loop_rate(4);
    geometry_msgs::Twist speed;
    int param_r=0,param_g=0,param_b=0;
    ROS_INFO("draw spiral start");

    while(ros::ok()) {
//读取参数，放入服务请求，发送请求
        if(node.getParam("/line_color/line_r",param_r)&&
        node.getParam("/line_color/line_g",param_g)&&
        node.getParam("/line_color/line_b",param_b))
            ROS_INFO("Got param, r:%d, g:%d, b:%d",param_r,param_g,param_b);
        else
            ROS_ERROR("Failed to get param 'line_color'");
        color_init.request.r=param_r;
        color_init.request.g=param_g;
        color_init.request.b=param_b;
        color_init.request.width=5;
        color_init.request.off=0;
        lineColorChange.call(color_init);
//执行回调函数
        ros::spinOnce();
//发布速度
        speed.linear.x=k_linear*0.2;
        speed.linear.y=0;
        speed.linear.z=0;
        speed.angular.x=0;
        speed.angular.y=0;
        speed.angular.z=2.0;
        ++k_linear;
        speedPub.publish(speed);
        loop_rate.sleep();
    }
    return 0;
}


