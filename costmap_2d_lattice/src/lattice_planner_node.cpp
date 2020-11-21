//
// Created by dlsh on 2020/11/20.
//

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include "dynamo_msgs/TeensyRead.h"
#include <visualization_msgs/Marker.h>

double theta_car,x_car,y_car,speed_car;
double steering_current;
double s_current[4];//0:x,1:y,2:theta,3:v

const double PI = 3.141592653589793238463;
const double Ts = 0.1,T_final=4;
const double L=1.516,max_deg_speed=5,max_rad_speed=5*PI/180,cof_engine=0.27;
double u_angle_deg_use[13]={-15.0,-12.5,-10.0,-7.5,-5.0,-2.5,0.0,2.5,5.0,7.5,10.0,12.5,15.0};
double u_angle_rad_use[13]={-0.261799,-0.218166,-0.17453,-0.130899,-0.087266,-0.043633,0.0,0.043633,0.087266,0.130899,0.17453,0.218166,0.261799};
double u_engine_use[3]={0,1,2};
double u_break_use[4]={0,5,10,15};


ros::Publisher pub_marker;

void fill_steering_input_array(double* array,double len_t_list,double steering_goal);
double** ode_model_predict(double* s0,double* t_list,int len_t_list,double* u_angle_list, double* u_engin_list,double* u_break_list);
void traj_vis(double** s_10l_fast,double** s_10r_fast,double** s_5l_fast,double** s_5r_fast,double** s_0_fast, int len_t_list);
void delete_s_array(double** s_array,int len_t_list);
void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom);
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg);

void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom) {
    // msg_odom[0]: x
    // msg_odom[1]: y
    // msg_odom[3]: theta [rad](orientation)
    s_current[0] = msg_odom->data[0];
    s_current[1] = msg_odom->data[1];
    s_current[2] = msg_odom->data[3];

    //calculate input
    //time input
    int len_t_list=T_final/Ts+1;
    double t_list[len_t_list];
    double u_angle_list_10l[len_t_list],u_angle_list_10r[len_t_list],u_angle_list_5l[len_t_list],u_angle_list_5r[len_t_list],u_angle_list_0[len_t_list];
    double u_engine_list_fast[len_t_list], u_engine_list_close[len_t_list];
    double u_break_list[len_t_list];
    //steering input
    fill_steering_input_array(u_angle_list_10l,len_t_list,0.17453);
    fill_steering_input_array(u_angle_list_10r,len_t_list,-0.17453);
    fill_steering_input_array(u_angle_list_5l,len_t_list,0.087266);
    fill_steering_input_array(u_angle_list_5r,len_t_list,-0.087266);
    fill_steering_input_array(u_angle_list_0,len_t_list,0);

    for(int i=0;i<len_t_list;++i){
        //time input
        t_list[i]=Ts*i;
        //break input
        u_break_list[i]=0.0;
        //engin input
        u_engine_list_fast[i]=2*cof_engine;
        u_engine_list_close[i]=0.0;
    }
    double** s_10l_fast=ode_model_predict(s_current,t_list,len_t_list,u_angle_list_10l,u_engine_list_fast,u_break_list);
    double** s_10r_fast=ode_model_predict(s_current,t_list,len_t_list,u_angle_list_10r,u_engine_list_fast,u_break_list);
    double** s_5l_fast=ode_model_predict(s_current,t_list,len_t_list,u_angle_list_5l,u_engine_list_fast,u_break_list);
    double** s_5r_fast=ode_model_predict(s_current,t_list,len_t_list,u_angle_list_5r,u_engine_list_fast,u_break_list);
    double** s_0_fast=ode_model_predict(s_current,t_list,len_t_list,u_angle_list_0,u_engine_list_fast,u_break_list);
    traj_vis(s_10l_fast,s_10r_fast,s_5l_fast,s_5r_fast,s_0_fast,len_t_list);
    delete_s_array(s_10l_fast,len_t_list);
    delete_s_array(s_10r_fast,len_t_list);
    delete_s_array(s_5l_fast,len_t_list);
    delete_s_array(s_5r_fast,len_t_list);
    delete_s_array(s_0_fast,len_t_list);
    steering_current=0;
}
void delete_s_array(double** s_array,int len_t_list){
    for(int i = 0; i < len_t_list; i++)
        delete[] s_array[i];
    delete[] s_array;
}
void fill_steering_input_array(double* array,double len_t_list,double steering_goal){
    //int len=abs(steering_goal-steering_current)/(max_deg_speed*Ts);
    array[0]=steering_current;
    for(int i=1;i<len_t_list;++i){
        if(array[i-1]>=steering_goal)
            array[i]=steering_goal;
        else {
            array[i] = steering_current + i * max_rad_speed * Ts;
        }
    }
}
double** ode_model_predict(double* s0,double* t_list,int len_t_list,double* u_angle_list, double* u_engin_list,double* u_break_list){
    double x=s0[0], y=s0[1],theta=s0[2],v=s0[3];
    double u_angle=u_angle_list[0],u_engin=u_engin_list[0],u_break=u_break_list[0];
    double v_last=v, theta_last=theta;

    //Apply for dynamic memory
    double **s_list = new double*[len_t_list];
    for(int i = 0; i < len_t_list; i++)
        s_list[i] = new double[4];

    //ode forward simulation
    for(int i=0;i<len_t_list;++i){
        u_angle=u_angle_list[i];
        u_engin=u_engin_list[i];
        u_break=u_break_list[i];
        v_last=v;
        theta_last=theta;
        theta=theta_last+Ts*(v_last/L)*tan(u_angle);
        v=v_last+Ts*(u_engin+u_break);
        x=x+Ts*v_last*cos(theta_last);
        y=y+Ts*v_last*sin(theta_last);
        s_list[i][0]=x;
        s_list[i][1]=y;
        s_list[i][2]=theta;
        s_list[i][3]=v;
    }
    return s_list;
}
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg) {
    s_current[3] = msg->speed_wheel;
}

void traj_vis(double** s_10l_fast,double** s_10r_fast,double** s_5l_fast,double** s_5r_fast,double** s_0_fast, int len_t_list) {
    visualization_msgs::Marker traj, points2, points3, points_line,points_line2;
    ros::Duration one_sec(.4);

    traj.header.frame_id = points2.header.frame_id = points3.header.frame_id =
    points_line.header.frame_id = points_line2.header.frame_id =
            "world"; //"base_link";
    traj.header.stamp = points2.header.stamp = points3.header.stamp =
    points_line.header.stamp = points_line2.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    traj.ns = "points";
    points2.ns = "points_corrected";
    points3.ns = "points_corrected_global";
    points_line.ns = "points_line";
    points_line2.ns = "points_line_debug";

    traj.action = points2.action = points3.action = points_line.action =
    points_line2.action = visualization_msgs::Marker::ADD;

    traj.pose.orientation.w = points2.pose.orientation.w =
    points3.pose.orientation.w = points_line.pose.orientation.w =
    points_line2.pose.orientation.w = 1.0;

    traj.id = 0;
    points2.id = 1;
    points3.id = 2;
    points_line.id = 3;
    points_line2.id = 4;

    // Set the marker type.

    traj.type = visualization_msgs::Marker::POINTS;
    points2.type = visualization_msgs::Marker::POINTS;
    points3.type = visualization_msgs::Marker::POINTS;
    points_line.type = visualization_msgs::Marker::POINTS;
    points_line2.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    traj.scale.x = 0.2;
    traj.scale.y = 0.2;

    points2.scale.x = 0.2;
    points2.scale.y = 0.2;

    points3.scale.x = 0.2;
    points3.scale.y = 0.2;

    points_line.scale.x = 0.2;
    points_line.scale.y = 0.2;

    points_line2.scale.x = 0.2;
    points_line2.scale.y = 0.2;

    traj.color.b = 1.0; // blue
    traj.color.a = 1.0;

    points2.color.g = 1.0f; // green
    points2.color.a = 1.0;

    points3.color.r = 1.0f; // red
    points3.color.a = 1.0;
/*
    if (fast)
        points_line.color.g = 1.0f; // green
    if (!fast)
        points_line.color.b = 1.0f; // blue
    if (bad == 1)
        points_line.color.b = points_line.color.g = 1.0f;
*/
    points_line.color.a = 1.0;

    points_line2.color.r = 1.0f; // red
    points_line2.color.a = 1.0;

    for (int i = 0; i < len_t_list; i++) {
        geometry_msgs::Point p_10l, p_10r, p_5l, p_5r, p_0;
        p_10l.x = s_10l_fast[i][0];
        p_10l.y = s_10l_fast[i][1];
        p_10l.z = 0;
        p_10r.x = s_10r_fast[i][0];
        p_10r.y = s_10r_fast[i][1];
        p_10r.z = 0;
        p_5l.x = s_5l_fast[i][0];
        p_5l.y = s_5l_fast[i][1];
        p_5l.z = 0;
        p_5r.x = s_5r_fast[i][0];
        p_5r.y = s_5r_fast[i][1];
        p_5r.z = 0;
        p_0.x = s_0_fast[i][0];
        p_0.y = s_0_fast[i][1];
        p_0.z = 0;
        traj.points.push_back(p_10l);
        traj.points.push_back(p_10r);
        traj.points.push_back(p_5l);
        traj.points.push_back(p_5r);
        traj.points.push_back(p_0);
    }
/*
    //DLSH ADD
    x_allMsg.clear();
    y_allMsg.clear();
    //DLSH
    double precision = 0.4; // distance between points
    // std::vector<Pos> poses = smooth(xvec,yvec,precision,0);
    std::vector<Pos> poses = posesGlobal;

    for (int i = 0; i < poses.size(); i++) {
        geometry_msgs::Point p2;
        p2.x = poses[i].x;
        p2.y = poses[i].y;
        p2.z = 0;
        points2.points.push_back(p2);
    }
    std::vector<Pos> poses2 = posesGlobalGlobal;
    for (int i = 2.9 / precision; i < poses2.size();
         i++) { // i=29, since starting from 3m ahead
        geometry_msgs::Point p2;
        p2.x = poses2[i].x;
        p2.y = poses2[i].y;
        p2.z = 0;
        points3.points.push_back(p2);
    }
/*
  for (int i = 0; i < poses_debug.size(); i++) {
    geometry_msgs::Point p2;
    // calculate points to global coordinates
    double globalx = cos(odotheta) * poses_debug[i].x -
                     sin(odotheta) * poses_debug[i].y + odox;
    double globaly = sin(odotheta) * poses_debug[i].x +
                     cos(odotheta) * poses_debug[i].y + odoy;

    p2.x = globalx;
    p2.y = globaly;
    p2.z = 0;

    points_line2.points.push_back(p2);
  }

//DLSH ADD
    for (int i = 0; i < xvec_global2.size(); i++) {
        geometry_msgs::Point p2;

        p2.x = xvec_global2[i];
        p2.y = yvec_global2[i];
        p2.z = 0;
        points_line2.points.push_back(p2);
    }
//DLSH
    // points_line
    std::vector<Pos> poses_line = posesGlobalGlobal;
    int amount = 15;
    double resol = 0.25;
    for (int i = 2.5 / resol; i < 2.5 / resol + amount; i++) {
        geometry_msgs::Point p2;
        p2.x = line_x + cos(line_th) * resol * i;
        p2.y = line_y + sin(line_th) * resol * i;
        p2.z = 0;
        points_line.points.push_back(p2);
    }*/
    // points.lifetime = points2.lifetime = points3.lifetime =
    // points_line.lifetime = one_sec;
    // points.lifetime = points2.lifetime = points3.lifetime =
    // points_line.lifetime = one_sec;
    pub_marker.publish(traj);
    //pub_marker.publish(points2);
    //pub_marker.publish(points3);
    //pub_marker.publish(points_line);
    //pub_marker.publish(points_line2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lattice_planner");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS lcr("costmap", buffer);

    ros::Subscriber sub_odom = nh.subscribe("car_pose_estimate", 1000, odomCallback);
    ros::Subscriber sub_teensyread = nh.subscribe("teensy_read", 10, teensyCallback);

    pub_marker = nh.advertise<visualization_msgs::Marker>("lattice_marker", 0);
    float loop_Rate = 20; //[Hz]
    ros::Rate r(loop_Rate);
    ROS_INFO("Main loop.");
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return (0);
}