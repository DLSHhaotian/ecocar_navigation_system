//
// Created by dlsh on 2020/11/20.
//

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include "dynamo_msgs/TeensyRead.h"
#include "auto_navi/motorMsg.h"
#include "dynamo_msgs/SteeringStepper.h"
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>

using namespace std;
double theta_car,x_car,y_car,speed_car;
double steering_current;
double s_current[4];//0:x,1:y,2:theta,3:v

const double PI = 3.141592653589793238463;
const double Ts = 0.1,T_final=2.5;
const double L=1.516,max_deg_speed=20,max_rad_speed=5*PI/180,cof_engine=0.27;//cof_engine=0.27;
const double speed_switch=4.0,speed_toofast=5.0;
const double A_front=0.8575,Cd=1.0,rho_air=1.15,m_car=130,m_all=200;

bool flag_slide=false;
double u_angle_deg_use[13]={-15.0,-12.5,-10.0,-7.5,-5.0,-2.5,0.0,2.5,5.0,7.5,10.0,12.5,15.0};
double u_angle_rad_use[13]={-0.261799,-0.218166,-0.17453,-0.130899,-0.087266,-0.043633,0.0,0.043633,0.087266,0.130899,0.17453,0.218166,0.261799};
double u_engine_use[3]={0,1,2};
double u_break_use[4]={0,5,10,15};
int traj_history_id=10;

const char* file="/home/dlsh/Ecocar_offline_path_optimization/result_analysis/mue=0.9/traj_opt.csv";
vector<float> road_x,road_y;
const double dis_find_global=10.0;//[m]
double drive_dist=0.0;

ros::Publisher pub_marker;
ros::Publisher motoPub;
ros::Publisher steerPub;
ros::Publisher mapSwitch;
void fill_steering_input_array(double* array,double len_t_list,double steering_goal);
double** ode_model_predict(double* s0,double* t_list,int len_t_list,double* u_angle_list, double* u_engin_list,double* u_break_list);
void traj_vis(double** s_15l,double** s_15r,double** s_12_5l,double** s_12_5r, double** s_10l,double** s_10r,double** s_7_5l,double** s_7_5r,double** s_5l,double** s_5r,double** s_2_5l,double** s_2_5r,double** s_0, double** s_min,int len_t_list, int index_global_goal);
void delete_s_array(double** s_array,int len_t_list);
void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom);
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg);
//double score_traj(costmap_2d::Costmap2D* map,double** s_array,int len_t_list);
double score_traj(costmap_2d::Costmap2DROS& map,double** s_array,int len_t_list, int& index_global_goal);
double score_traj_only_global(costmap_2d::Costmap2DROS& costmap_ros,double** s_array,int len_t_list, int& index_global_goal);
void lattice_planner(costmap_2d::Costmap2DROS& costmap_ros);
int switch_map(costmap_2d::Costmap2DROS& costmap_ros);




void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom) {
    // msg_odom[0]: x
    // msg_odom[1]: y
    // msg_odom[3]: theta [rad](orientation)
    // msg_odom[6]: drive dist
    s_current[0] = msg_odom->data[0];
    s_current[1] = msg_odom->data[1];
    s_current[2] = msg_odom->data[3];
    drive_dist = msg_odom->data[6];
}
void delete_s_array(double** s_array,int len_t_list){
    for(int i = 0; i < len_t_list; i++)
        delete[] s_array[i];//删除s_array每一行的子数组的指针，每个指针都指向容量为5的子数组
    delete[] s_array;//删除大指针，指向第一个子数组，同时也是指向整个二维数组
    s_array= nullptr;
}
void fill_steering_input_array(double* array,double len_t_list,double steering_goal){
    //int len=abs(steering_goal-steering_current)/(max_deg_speed*Ts);
    double sign_goal;
    if((steering_goal-steering_current)>0)
        sign_goal=1.0;
    else if((steering_goal-steering_current)<0)
        sign_goal=-1.0;
    else
        sign_goal=0.0;
    array[0]=steering_current;
    for(int i=1;i<len_t_list;++i){
        if(abs(array[i-1])>=abs(steering_goal))
            array[i]=steering_goal;
        else {
            array[i] = steering_current + sign_goal*i * max_rad_speed * Ts;
        }
    }
}
double** ode_model_predict(double* s0,double* t_list,int len_t_list,double* u_angle_list, double* u_engin_list,double* u_break_list){
    double x=s0[0], y=s0[1],theta=s0[2],v=s0[3];
    double u_angle=u_angle_list[0],u_engin=u_engin_list[0],u_break=u_break_list[0];
    double v_last=v, theta_last=theta;
    double a_drag=0.0;
    //Apply for dynamic memory
    double **s_list = new double*[len_t_list];//先new一个大指针，包含了所有数量的子数组，子数组具体多大不管，关注的是有多少行
    for(int i = 0; i < len_t_list; i++)
        s_list[i] = new double[5];//遍历每个小指针，new固定数量的子数组给这个小指针

    //ode forward simulation
    for(int i=0;i<len_t_list;++i){
        u_angle=u_angle_list[i];
        u_engin=u_engin_list[i];
        u_break=u_break_list[i];
        v_last=v;
        a_drag=-0.5*A_front*Cd*rho_air*pow(v_last,2)/m_car;
        theta_last=theta;
        theta=theta_last+Ts*(v_last/L)*tan(u_angle);
        v=v_last+Ts*(u_engin+u_break+a_drag);
        x=x+Ts*v_last*cos(theta_last);
        y=y+Ts*v_last*sin(theta_last);
        s_list[i][0]=x;
        s_list[i][1]=y;
        s_list[i][2]=theta;
        s_list[i][3]=v;
        s_list[i][4]=u_angle;
    }
    return s_list;
}
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg) {
    s_current[3] = msg->speed_wheel;
}

void traj_vis(double** s_15l,double** s_15r,double** s_12_5l,double** s_12_5r, double** s_10l,double** s_10r,double** s_7_5l,double** s_7_5r,double** s_5l,double** s_5r,double** s_2_5l,double** s_2_5r,double** s_0, double** s_min,int len_t_list, int index_global_goal) {
    visualization_msgs::Marker traj, traj_min, traj_used, traj_input,traj_global;
    ros::Duration one_sec(.4);

    traj.header.frame_id = traj_min.header.frame_id = traj_used.header.frame_id =
    traj_input.header.frame_id = traj_global.header.frame_id =
            "world"; //"base_link";
    traj.header.stamp = traj_min.header.stamp = traj_used.header.stamp =
    traj_input.header.stamp = traj_global.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    traj.ns = "points";
    traj_min.ns = "points_corrected";
    traj_used.ns = "points_corrected_global";
    traj_input.ns = "points_line";
    traj_global.ns = "points_line_debug";

    traj.action = traj_min.action = traj_used.action = traj_input.action =
    traj_global.action = visualization_msgs::Marker::ADD;

    traj.pose.orientation.w = traj_min.pose.orientation.w =
    traj_used.pose.orientation.w = traj_input.pose.orientation.w =
    traj_global.pose.orientation.w = 1.0;

    traj.id = 0;
    traj_min.id = 1;
    //traj_used.id = 2;
    traj_input.id = traj_history_id+500;
    traj_global.id = 4;
    traj_used.id=traj_history_id;
    ++traj_history_id;

    //set the lifetime
    traj_used.lifetime=ros::Duration(0.0);
    traj_input.lifetime=ros::Duration(0.0);
    // Set the marker type.

    traj.type = visualization_msgs::Marker::POINTS;
    traj_min.type = visualization_msgs::Marker::POINTS;
    traj_used.type = visualization_msgs::Marker::POINTS;
    traj_input.type = visualization_msgs::Marker::POINTS;
    traj_global.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    traj.scale.x = 0.2;
    traj.scale.y = 0.2;

    traj_min.scale.x = 0.2;
    traj_min.scale.y = 0.2;

    traj_used.scale.x = 0.2;
    traj_used.scale.y = 0.2;

    traj_input.scale.x = 0.2;
    traj_input.scale.y = 0.2;

    traj_global.scale.x = 0.5;
    traj_global.scale.y = 0.5;

    traj.color.b = 1.0; // blue
    traj.color.a = 1.0;

    traj_min.color.g = 1.0f; // green
    traj_min.color.a = 1.0;

    traj_used.color.r = 1.0f; // red
    traj_used.color.a = 1.0;

    traj_input.color.r = 0.5; //brown
    traj_input.color.a = 1.0;

    traj_global.color.g = 0.5; // red
    traj_global.color.a = 1.0;

    for (int i = 0; i < len_t_list; i++) {
        geometry_msgs::Point p_15l, p_15r, p_12_5l, p_12_5r,p_10l, p_10r, p_7_5l, p_7_5r, p_5l, p_5r,p_2_5l, p_2_5r, p_0;
        p_15l.x = s_15l[i][0];
        p_15l.y = s_15l[i][1];
        p_15l.z = 0;
        p_15r.x = s_15r[i][0];
        p_15r.y = s_15r[i][1];
        p_15r.z = 0;
        p_12_5l.x = s_12_5l[i][0];
        p_12_5l.y = s_12_5l[i][1];
        p_12_5l.z = 0;
        p_12_5r.x = s_12_5r[i][0];
        p_12_5r.y = s_12_5r[i][1];
        p_12_5r.z = 0;
        p_10l.x = s_10l[i][0];
        p_10l.y = s_10l[i][1];
        p_10l.z = 0;
        p_10r.x = s_10r[i][0];
        p_10r.y = s_10r[i][1];
        p_10r.z = 0;
        p_7_5l.x = s_7_5l[i][0];
        p_7_5l.y = s_7_5l[i][1];
        p_7_5l.z = 0;
        p_7_5r.x = s_7_5r[i][0];
        p_7_5r.y = s_7_5r[i][1];
        p_7_5r.z = 0;
        p_5l.x = s_5l[i][0];
        p_5l.y = s_5l[i][1];
        p_5l.z = 0;
        p_5r.x = s_5r[i][0];
        p_5r.y = s_5r[i][1];
        p_5r.z = 0;
        p_2_5l.x = s_2_5l[i][0];
        p_2_5l.y = s_2_5l[i][1];
        p_2_5l.z = 0;
        p_2_5r.x = s_2_5r[i][0];
        p_2_5r.y = s_2_5r[i][1];
        p_2_5r.z = 0;
        p_0.x = s_0[i][0];
        p_0.y = s_0[i][1];
        p_0.z = 0;
        traj.points.push_back(p_15l);
        traj.points.push_back(p_15r);
        traj.points.push_back(p_12_5l);
        traj.points.push_back(p_12_5r);
        traj.points.push_back(p_10l);
        traj.points.push_back(p_10r);
        traj.points.push_back(p_7_5l);
        traj.points.push_back(p_7_5r);
        traj.points.push_back(p_5l);
        traj.points.push_back(p_5r);
        traj.points.push_back(p_2_5l);
        traj.points.push_back(p_2_5r);
        traj.points.push_back(p_0);
    }
    for(int i=0;i<len_t_list;++i){
        geometry_msgs::Point p_min;
        p_min.x=s_min[i][0];
        p_min.y=s_min[i][1];
        p_min.z=0;
        traj_min.points.push_back(p_min);
    }
    for(int i=0;i<len_t_list/3;++i){
        geometry_msgs::Point p_used;
        p_used.x=s_min[i][0];
        p_used.y=s_min[i][1];
        p_used.z=0;
        traj_used.points.push_back(p_used);
    }


    geometry_msgs::Point p_input;
    p_input.x=s_min[4][0];
    p_input.y=s_min[4][1];
    p_input.z=0;
    traj_input.points.push_back(p_input);

    geometry_msgs::Point p_global;
    p_global.x=road_x[index_global_goal];
    p_global.y=road_y[index_global_goal];
    p_global.z=0;
    traj_global.points.push_back(p_global);

    pub_marker.publish(traj);
    pub_marker.publish(traj_min);
    pub_marker.publish(traj_used);
    pub_marker.publish(traj_input);
    pub_marker.publish(traj_global);
}
void lattice_planner(costmap_2d::Costmap2DROS& costmap_ros){
    auto_navi::motorMsg max_motor;
    dynamo_msgs::SteeringStepper steeringmsg;
    std_msgs::Int8 mapSwichmsg;

    double s_current_temp[4];
    for (int i=0;i<4;++i){
        s_current_temp[i]=s_current[i];
    }
    //calculate input
    //time input
    int len_t_list=T_final/Ts+1;
    double t_list[len_t_list];
    double u_angle_list_15l[len_t_list],u_angle_list_15r[len_t_list],u_angle_list_12_5l[len_t_list],u_angle_list_12_5r[len_t_list],u_angle_list_10l[len_t_list],u_angle_list_10r[len_t_list],u_angle_list_7_5l[len_t_list],u_angle_list_7_5r[len_t_list],u_angle_list_5l[len_t_list],u_angle_list_5r[len_t_list],u_angle_list_2_5l[len_t_list],u_angle_list_2_5r[len_t_list],u_angle_list_0[len_t_list];
    double u_engine_list[len_t_list];
    double u_break_list[len_t_list];
    //steering input
    fill_steering_input_array(u_angle_list_15l,len_t_list,0.261799);
    fill_steering_input_array(u_angle_list_15r,len_t_list,-0.261799);
    fill_steering_input_array(u_angle_list_12_5l,len_t_list,0.218166);
    fill_steering_input_array(u_angle_list_12_5r,len_t_list,-0.218166);
    fill_steering_input_array(u_angle_list_10l,len_t_list,0.17453);
    fill_steering_input_array(u_angle_list_10r,len_t_list,-0.17453);
    fill_steering_input_array(u_angle_list_7_5l,len_t_list,0.130899);
    fill_steering_input_array(u_angle_list_7_5r,len_t_list,-0.130899);
    fill_steering_input_array(u_angle_list_5l,len_t_list,0.087266);
    fill_steering_input_array(u_angle_list_5r,len_t_list,-0.087266);
    fill_steering_input_array(u_angle_list_2_5l,len_t_list,0.043633);
    fill_steering_input_array(u_angle_list_2_5r,len_t_list,-0.043633);
    fill_steering_input_array(u_angle_list_0,len_t_list,0);


    bool gear_change=s_current_temp[3]>speed_switch;
    bool engin_close=s_current_temp[3]>speed_toofast;
    for(int i=0;i<len_t_list;++i){
        //time input
        t_list[i]=Ts*i;
        //break input
        u_break_list[i]=0.0;
        //engin input
        if(gear_change&&!engin_close&&!flag_slide){
            u_engine_list[i]=1*cof_engine;}
        else if(engin_close) {
            u_engine_list[i] = 0.0;
            flag_slide=true;
        }
        else{
            u_engine_list[i]=2*cof_engine;
            flag_slide=false;
        }
    }
    //generate the traj using the model simulation
    //s_list[0]:x
    //      [1]:y
    //      [2]:theta
    //      [3]:v
    //      [4]:u_angle(not the state but need to be recorded)
    double** s_15l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_15l,u_engine_list,u_break_list);
    double** s_15r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_15r,u_engine_list,u_break_list);
    double** s_12_5l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_12_5l,u_engine_list,u_break_list);
    double** s_12_5r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_12_5r,u_engine_list,u_break_list);
    double** s_10l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_10l,u_engine_list,u_break_list);
    double** s_10r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_10r,u_engine_list,u_break_list);
    double** s_7_5l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_7_5l,u_engine_list,u_break_list);
    double** s_7_5r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_7_5r,u_engine_list,u_break_list);
    double** s_5l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_5l,u_engine_list,u_break_list);
    double** s_5r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_5r,u_engine_list,u_break_list);
    double** s_2_5l=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_2_5l,u_engine_list,u_break_list);
    double** s_2_5r=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_2_5r,u_engine_list,u_break_list);
    double** s_0=ode_model_predict(s_current_temp,t_list,len_t_list,u_angle_list_0,u_engine_list,u_break_list);
    //score the traj
    int index_global_goal=(drive_dist+dis_find_global)/1.0;
    //we have to wait until the map is updated
    ros::Rate r(100.0);
    while (ros::ok() && !costmap_ros.isInitialized())
        r.sleep();
    double score_15l=score_traj(costmap_ros,s_15l,len_t_list,index_global_goal);
    double score_15r=score_traj(costmap_ros,s_15r,len_t_list,index_global_goal);
    double score_12_5l=score_traj(costmap_ros,s_12_5l,len_t_list,index_global_goal);
    double score_12_5r=score_traj(costmap_ros,s_12_5r,len_t_list,index_global_goal);
    double score_10l=score_traj(costmap_ros,s_10l,len_t_list,index_global_goal);
    double score_10r=score_traj(costmap_ros,s_10r,len_t_list,index_global_goal);
    double score_7_5l=score_traj(costmap_ros,s_7_5l,len_t_list,index_global_goal);
    double score_7_5r=score_traj(costmap_ros,s_7_5r,len_t_list,index_global_goal);
    double score_5l=score_traj(costmap_ros,s_5l,len_t_list,index_global_goal);
    double score_5r=score_traj(costmap_ros,s_5r,len_t_list,index_global_goal);
    double score_2_5l=score_traj(costmap_ros,s_2_5l,len_t_list,index_global_goal);
    double score_2_5r=score_traj(costmap_ros,s_2_5r,len_t_list,index_global_goal);
    double score_0=score_traj(costmap_ros,s_0,len_t_list,index_global_goal);

    multimap<double,double**> score_s_list;
    score_s_list.insert(make_pair(score_15l,s_15l));
    score_s_list.insert(make_pair(score_15r,s_15r));
    score_s_list.insert(make_pair(score_12_5l,s_12_5l));
    score_s_list.insert(make_pair(score_12_5r,s_12_5r));
    score_s_list.insert(make_pair(score_10l,s_10l));
    score_s_list.insert(make_pair(score_10r,s_10r));
    score_s_list.insert(make_pair(score_7_5l,s_7_5l));
    score_s_list.insert(make_pair(score_7_5r,s_7_5r));
    score_s_list.insert(make_pair(score_5l,s_5l));
    score_s_list.insert(make_pair(score_5r,s_5r));
    score_s_list.insert(make_pair(score_2_5l,s_2_5l));
    score_s_list.insert(make_pair(score_2_5r,s_2_5r));
    score_s_list.insert(make_pair(score_0,s_0));

    if(score_s_list.count(DBL_MAX)>6) {
        max_motor.fast = 0;//fast:0 ->>>>use the pair of slow speed fast:1 ->>>>use the pair of fast speed
    }
    else{
        max_motor.fast= 1;
    }

    double score_min=score_s_list.begin()->first;
    double** traj_select;
    typedef std::multimap<double,double**>::iterator multimap_iterator;

    if(score_min==0.0 && score_s_list.count(score_min)<13) {
        ROS_INFO("more than 1 traj's score is 0");
        pair<multimap_iterator,multimap_iterator> it =score_s_list.equal_range(score_min);
        while (it.first!=it.second)
        {
            it.first=score_s_list.erase(it.first);
        }
        traj_select=score_s_list.begin()->second;
    }
    else if(score_s_list.count(score_min)>=13){
        if(score_min==DBL_MAX||score_min==0.0){
            ROS_INFO("all traj is score DBL_MAX, the score will be recomputed only based on the global goal");
            score_15l=score_traj_only_global(costmap_ros,s_15l,len_t_list,index_global_goal);
            score_15r=score_traj_only_global(costmap_ros,s_15r,len_t_list,index_global_goal);
            score_12_5l=score_traj_only_global(costmap_ros,s_12_5l,len_t_list,index_global_goal);
            score_12_5r=score_traj(costmap_ros,s_12_5r,len_t_list,index_global_goal);
            score_10l=score_traj_only_global(costmap_ros,s_10l,len_t_list,index_global_goal);
            score_10r=score_traj_only_global(costmap_ros,s_10r,len_t_list,index_global_goal);
            score_7_5l=score_traj_only_global(costmap_ros,s_7_5l,len_t_list,index_global_goal);
            score_7_5r=score_traj_only_global(costmap_ros,s_7_5r,len_t_list,index_global_goal);
            score_5l=score_traj_only_global(costmap_ros,s_5l,len_t_list,index_global_goal);
            score_5r=score_traj_only_global(costmap_ros,s_5r,len_t_list,index_global_goal);
            score_2_5l=score_traj_only_global(costmap_ros,s_2_5l,len_t_list,index_global_goal);
            score_2_5r=score_traj_only_global(costmap_ros,s_2_5r,len_t_list,index_global_goal);
            score_0=score_traj_only_global(costmap_ros,s_0,len_t_list,index_global_goal);

            multimap<double,double**> score_s_list_global;
            score_s_list_global.insert(make_pair(score_15l,s_15l));
            score_s_list_global.insert(make_pair(score_15r,s_15r));
            score_s_list_global.insert(make_pair(score_12_5l,s_12_5l));
            score_s_list_global.insert(make_pair(score_12_5r,s_12_5r));
            score_s_list_global.insert(make_pair(score_10l,s_10l));
            score_s_list_global.insert(make_pair(score_10r,s_10r));
            score_s_list_global.insert(make_pair(score_7_5l,s_7_5l));
            score_s_list_global.insert(make_pair(score_7_5r,s_7_5r));
            score_s_list_global.insert(make_pair(score_5l,s_5l));
            score_s_list_global.insert(make_pair(score_5r,s_5r));
            score_s_list_global.insert(make_pair(score_2_5l,s_2_5l));
            score_s_list_global.insert(make_pair(score_2_5r,s_2_5r));
            score_s_list_global.insert(make_pair(score_0,s_0));
            traj_select=score_s_list_global.begin()->second;
            max_motor.fast = 0;
        }
        else{
            ROS_INFO("all traj is score 0,skip this time,use 0 steering as selected traj");
            traj_select=s_0;
        }

    }
    else{
        ROS_INFO("Min traj found");
        traj_select=score_s_list.begin()->second;
    }

    std::cout<<"15l:"<<score_15l<<"\n"<< "15r:"<<score_15r<<"\n" << "12.5l:"<<score_12_5l<<"\n" << "12.5r:"<<score_12_5r<<"\n"<<"10l:"<<score_10l<<"\n"<< "10r:"<<score_10r<<"\n" << "7.5l:"<<score_7_5l<<"\n" << "7.5r:"<<score_7_5r<<"\n"<< "5l:"<<score_5l<<"\n"<< "5r:"<<score_5r<<"\n"<< "2.5l:"<<score_2_5l<<"\n" << "2.5r:"<<score_2_5r<<"\n"<< "0:"<<score_0<<std::endl;

    traj_vis(s_15l,s_15r,s_12_5l,s_12_5r,s_10l,s_10r,s_7_5l,s_7_5r,s_5l,s_5r,s_2_5l,s_2_5r,s_0,traj_select,len_t_list,index_global_goal);
    //considering the computing time, the offset should be set
    steering_current=traj_select[5][4];
    ROS_INFO("STEERING INPUT:%f",steering_current);
    steeringmsg.steering_angle = steering_current*180/PI;
    steeringmsg.steering_stepper_engaged = 1;


    mapSwichmsg.data=switch_map(costmap_ros);
    //mapSwichmsg.data=0;
    mapSwitch.publish(mapSwichmsg);
    motoPub.publish(max_motor);
    steerPub.publish(steeringmsg);

    delete_s_array(s_15l,len_t_list);
    delete_s_array(s_15r,len_t_list);
    delete_s_array(s_12_5l,len_t_list);
    delete_s_array(s_12_5r,len_t_list);
    delete_s_array(s_10l,len_t_list);
    delete_s_array(s_10r,len_t_list);
    delete_s_array(s_7_5l,len_t_list);
    delete_s_array(s_7_5r,len_t_list);
    delete_s_array(s_5l,len_t_list);
    delete_s_array(s_5r,len_t_list);
    delete_s_array(s_2_5l,len_t_list);
    delete_s_array(s_2_5r,len_t_list);
    delete_s_array(s_0,len_t_list);
    traj_select= nullptr;

}
double score_traj(costmap_2d::Costmap2DROS& costmap_ros,double** s_array,int len_t_list, int& index_global_goal){
    double score=0.0,cost_obstacle=0.0;
    bool trans_succ=true;
    unsigned int x_map=0,y_map=0;
    double cost_global=0.0;
    if (index_global_goal>road_x.size())
        index_global_goal=road_x.size();
    double x_global_goal=road_x[index_global_goal];
    double y_global_goal=road_y[index_global_goal];
    costmap_2d::Costmap2D map=*(costmap_ros.getCostmap());
    //cout<<"initialized:"<<costmap_ros.getLayeredCostmap()->isInitialized()<<endl;
    for(int i=0;i<len_t_list;++i){
        trans_succ=map.worldToMap(s_array[i][0],s_array[i][1],x_map,y_map);
        if(trans_succ) {
            cost_obstacle=static_cast<double>(map.getCost(x_map, y_map));
            //ROS_INFO("cost: %f",cost);
            if (cost_obstacle==254.0){
                score=DBL_MAX;
                break;
            }
            score += cost_obstacle/100.0;
        }
        cost_global+=(pow(s_array[i][0]-x_global_goal,2)+pow(s_array[i][1]-y_global_goal,2));
        //cost_global+=(pow(s_array[i][0]-x_global_goal,2)+pow(s_array[i][1]-y_global_goal,2));
    }
    ROS_INFO("cost_obstacle:#####  %f", score);
    if(score!=DBL_MAX){
        score+=sqrt(cost_global/len_t_list);
        ROS_INFO("cost_global:#####  %f", sqrt(cost_global/len_t_list));
    }
    return score;
}
double score_traj_only_global(costmap_2d::Costmap2DROS& costmap_ros,double** s_array,int len_t_list, int& index_global_goal){
    double score=0.0;
    double cost_global=0.0;
    if (index_global_goal>road_x.size())
        index_global_goal=road_x.size();
    float x_global_goal=road_x[index_global_goal];
    float y_global_goal=road_y[index_global_goal];
    costmap_2d::Costmap2D map=*(costmap_ros.getCostmap());
    for(int i=0;i<len_t_list;++i){
        cost_global+=(pow(s_array[i][0]-x_global_goal,2)+pow(s_array[i][1]-y_global_goal,2));
    }
    score+=sqrt(cost_global/len_t_list);
    ROS_INFO("cost_global:#####  %f", sqrt(cost_global/len_t_list));
    return score;
}

int switch_map(costmap_2d::Costmap2DROS& costmap_ros){
    double map_width1=22.0,map_height1=16.0,map_width2=16.0,map_height2=22.0;
    double carpose_theta=s_current[2];
    int switchMap=0;
    int flag_ahead_rear=0;//1: x,ahead 2:x,rear 3:y,ahead 4:y,rear
    int index_check_free_x1=0,index_check_free_x2=0;//for pattern1
    int index_check_free_y1=0,index_check_free_y2=0;//for pattern2
    int num_free_1=0,num_free_2=0,num_threshold=10;

    //limit the theta into [-3.14 3.14)
    while(carpose_theta<-3.14){
        carpose_theta=carpose_theta+6.28;
    }
    while(carpose_theta>=3.14){
        carpose_theta=carpose_theta-6.28;
    }

    //check the map direction by car's orientation
    if(carpose_theta>=-0.785&&carpose_theta<=0.785){
        flag_ahead_rear=1;
    }
    else if((carpose_theta<=-2.355)||(carpose_theta>=2.355)){
        flag_ahead_rear=2;
    }
    else if((carpose_theta<=2.355)&&(carpose_theta>=0.785)){
        flag_ahead_rear=3;
    }
    else{
        flag_ahead_rear=4;
    }


    if(costmap_ros.getCostmap()->getSizeInMetersX()>map_height1){
        //map size pattern1
        //check the ahead of the map
        if(flag_ahead_rear==1) {
            index_check_free_x1 = costmap_ros.getCostmap()->getSizeInCellsX() * 5 / 6;
            for (int i = 0; i < costmap_ros.getCostmap()->getSizeInCellsY(); ++i) {
                if (costmap_ros.getCostmap()->getCost(index_check_free_x1, i) == 0) {
                    num_free_1++;
                }
            }
            index_check_free_x2 = index_check_free_x1 + 1;
            for (int i = 0; i < costmap_ros.getCostmap()->getSizeInCellsY(); ++i) {
                if (costmap_ros.getCostmap()->getCost(index_check_free_x2, i) == 0) {
                    num_free_2++;
                }
            }
        }
        else{
            //check the rear of the map
            index_check_free_x1=costmap_ros.getCostmap()->getSizeInCellsX()*1/6;
            for(int i=0;i<costmap_ros.getCostmap()->getSizeInCellsY();++i){
                if(costmap_ros.getCostmap()->getCost(index_check_free_x1,i)==0){
                    num_free_1++;
                }
            }
            index_check_free_x2=index_check_free_x1-1;
            for(int i=0;i<costmap_ros.getCostmap()->getSizeInCellsY();++i){
                if(costmap_ros.getCostmap()->getCost(index_check_free_x2,i)==0){
                    num_free_2++;
                }
            }
        }
        if((num_free_1>num_threshold&&num_free_2>num_threshold))
            switchMap=1;
    }
    else{
        //map size pattern2
        //check the ahead of the map
        if(flag_ahead_rear==3){
            index_check_free_y1=costmap_ros.getCostmap()->getSizeInCellsY()*5/6;
            for(int i=0;i<costmap_ros.getCostmap()->getSizeInCellsX();++i){
                if(costmap_ros.getCostmap()->getCost(i,index_check_free_y1)==0){
                    num_free_1++;
                }
            }
            index_check_free_y2=index_check_free_y1+1;
            for(int i=0;i<costmap_ros.getCostmap()->getSizeInCellsX();++i){
                if(costmap_ros.getCostmap()->getCost(i,index_check_free_y2)==0){
                    num_free_2++;
                }
            }
        }
        else {
            //check the rear of the map
            index_check_free_y1 = costmap_ros.getCostmap()->getSizeInCellsY() * 1 / 6;
            for (int i = 0; i < costmap_ros.getCostmap()->getSizeInCellsX(); ++i) {
                if (costmap_ros.getCostmap()->getCost(i, index_check_free_y1) == 0) {
                    num_free_1++;
                }
            }
            index_check_free_y2 = index_check_free_y1 - 1;
            for (int i = 0; i < costmap_ros.getCostmap()->getSizeInCellsX(); ++i) {
                if (costmap_ros.getCostmap()->getCost(i, index_check_free_y2) == 0) {
                    num_free_2++;
                }
            }
        }
        if((num_free_1>num_threshold&&num_free_2>num_threshold))
            switchMap=1;
    }
    ROS_INFO("if it is rear or head: #######%d",flag_ahead_rear);
    ROS_INFO("how many free grids for ahead 5/6:###### %d",num_free_1);
    ROS_INFO("how many free grids for rear 1/6-1:###### %d",num_free_2);
    return switchMap;
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

    mapSwitch = nh.advertise<std_msgs::Int8>("map_switch", 1);
    pub_marker = nh.advertise<visualization_msgs::Marker>("lattice_marker", 0);
    motoPub = nh.advertise<auto_navi::motorMsg>("fast", 1);
    steerPub = nh.advertise<dynamo_msgs::SteeringStepper>("cmd_steering_angle_goal", 100);

    float x=0.0,y=0.0;
    FILE *fp;
    fp=fopen(file,"r");
    while(1){
        fscanf(fp,"%f,%f",&x,&y);
        road_x.push_back(x);
        road_y.push_back(y);
        if (feof(fp)) break;
    }
    fclose(fp);


    float loop_Rate = 5; //[Hz]
    ros::Rate r(loop_Rate);
    ROS_INFO("Main loop.");
    while (ros::ok()) {
        ros::spinOnce();
        lattice_planner(lcr);
        r.sleep();
    }

    return (0);
}
