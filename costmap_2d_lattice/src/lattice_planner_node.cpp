//
// Created by dlsh on 2020/11/20.
//

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include "dynamo_msgs/TeensyRead.h"
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <vector>
#include <map>

using namespace std;
double theta_car,x_car,y_car,speed_car;
double steering_current;
double s_current[4];//0:x,1:y,2:theta,3:v

const double PI = 3.141592653589793238463;
const double Ts = 0.1,T_final=2.0;
const double L=1.516,max_deg_speed=5,max_rad_speed=5*PI/180,cof_engine=0.27;//cof_engine=0.27;
const double speed_switch=4.0,speed_toofast=5.0;
const double A_front=0.8575,Cd=1.0,rho_air=1.15,m_car=130,m_all=200;

bool flag_slide=false;
double u_angle_deg_use[13]={-15.0,-12.5,-10.0,-7.5,-5.0,-2.5,0.0,2.5,5.0,7.5,10.0,12.5,15.0};
double u_angle_rad_use[13]={-0.261799,-0.218166,-0.17453,-0.130899,-0.087266,-0.043633,0.0,0.043633,0.087266,0.130899,0.17453,0.218166,0.261799};
double u_engine_use[3]={0,1,2};
double u_break_use[4]={0,5,10,15};


ros::Publisher pub_marker;

void fill_steering_input_array(double* array,double len_t_list,double steering_goal);
double** ode_model_predict(double* s0,double* t_list,int len_t_list,double* u_angle_list, double* u_engin_list,double* u_break_list);
void traj_vis(double** s_10l,double** s_10r,double** s_7_5l,double** s_7_5r,double** s_5l,double** s_5r,double** s_2_5l,double** s_2_5r,double** s_0, double** s_min,int len_t_list);
void delete_s_array(double** s_array,int len_t_list);
void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom);
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg);
double score_traj(costmap_2d::Costmap2DROS& costmap_ros,double** s_array,int len_t_list);
void lattice_planner(costmap_2d::Costmap2DROS& costmap_ros);




void odomCallback(const std_msgs::Float32MultiArray::ConstPtr &msg_odom) {
    // msg_odom[0]: x
    // msg_odom[1]: y
    // msg_odom[3]: theta [rad](orientation)
    s_current[0] = msg_odom->data[0];
    s_current[1] = msg_odom->data[1];
    s_current[2] = msg_odom->data[3];
}
void delete_s_array(double** s_array,int len_t_list){
    for(int i = 0; i < len_t_list; i++)
        delete[] s_array[i];
    delete[] s_array;
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
    double **s_list = new double*[len_t_list];
    for(int i = 0; i < len_t_list; i++)
        s_list[i] = new double[4];

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
    }
    return s_list;
}
void teensyCallback(const dynamo_msgs::TeensyReadPtr &msg) {
    s_current[3] = msg->speed_wheel;
}

void traj_vis(double** s_10l,double** s_10r,double** s_7_5l,double** s_7_5r,double** s_5l,double** s_5r,double** s_2_5l,double** s_2_5r,double** s_0, double** s_min,int len_t_list) {
    visualization_msgs::Marker traj, traj_min, points3, points_line,points_line2;
    ros::Duration one_sec(.4);

    traj.header.frame_id = traj_min.header.frame_id = points3.header.frame_id =
    points_line.header.frame_id = points_line2.header.frame_id =
            "world"; //"base_link";
    traj.header.stamp = traj_min.header.stamp = points3.header.stamp =
    points_line.header.stamp = points_line2.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    traj.ns = "points";
    traj_min.ns = "points_corrected";
    points3.ns = "points_corrected_global";
    points_line.ns = "points_line";
    points_line2.ns = "points_line_debug";

    traj.action = traj_min.action = points3.action = points_line.action =
    points_line2.action = visualization_msgs::Marker::ADD;

    traj.pose.orientation.w = traj_min.pose.orientation.w =
    points3.pose.orientation.w = points_line.pose.orientation.w =
    points_line2.pose.orientation.w = 1.0;

    traj.id = 0;
    traj_min.id = 1;
    points3.id = 2;
    points_line.id = 3;
    points_line2.id = 4;

    // Set the marker type.

    traj.type = visualization_msgs::Marker::POINTS;
    traj_min.type = visualization_msgs::Marker::POINTS;
    points3.type = visualization_msgs::Marker::POINTS;
    points_line.type = visualization_msgs::Marker::POINTS;
    points_line2.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    traj.scale.x = 0.2;
    traj.scale.y = 0.2;

    traj_min.scale.x = 0.2;
    traj_min.scale.y = 0.2;

    points3.scale.x = 0.2;
    points3.scale.y = 0.2;

    points_line.scale.x = 0.2;
    points_line.scale.y = 0.2;

    points_line2.scale.x = 0.2;
    points_line2.scale.y = 0.2;

    traj.color.b = 1.0; // blue
    traj.color.a = 1.0;

    traj_min.color.g = 1.0f; // green
    traj_min.color.a = 1.0;

    points3.color.r = 1.0f; // red
    points3.color.a = 1.0;

    points_line.color.a = 1.0;

    points_line2.color.r = 1.0f; // red
    points_line2.color.a = 1.0;

    for (int i = 0; i < len_t_list; i++) {
        geometry_msgs::Point p_10l, p_10r, p_7_5l, p_7_5r, p_5l, p_5r,p_2_5l, p_2_5r, p_0;
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

    pub_marker.publish(traj);
    pub_marker.publish(traj_min);
    //pub_marker.publish(points3);
    //pub_marker.publish(points_line);
    //pub_marker.publish(points_line2);
}
void lattice_planner(costmap_2d::Costmap2DROS& costmap_ros){
    double s_current_temp[4];
    for (int i=0;i<4;++i){
        s_current_temp[i]=s_current[i];
    }
    //calculate input
    //time input
    int len_t_list=T_final/Ts+1;
    double t_list[len_t_list];
    double u_angle_list_10l[len_t_list],u_angle_list_10r[len_t_list],u_angle_list_7_5l[len_t_list],u_angle_list_7_5r[len_t_list],u_angle_list_5l[len_t_list],u_angle_list_5r[len_t_list],u_angle_list_2_5l[len_t_list],u_angle_list_2_5r[len_t_list],u_angle_list_0[len_t_list];
    double u_engine_list[len_t_list];
    double u_break_list[len_t_list];
    //steering input
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
    double score_10l=score_traj(costmap_ros,s_10l,len_t_list);
    double score_10r=score_traj(costmap_ros,s_10r,len_t_list);
    double score_7_5l=score_traj(costmap_ros,s_7_5l,len_t_list);
    double score_7_5r=score_traj(costmap_ros,s_7_5r,len_t_list);
    double score_5l=score_traj(costmap_ros,s_5l,len_t_list);
    double score_5r=score_traj(costmap_ros,s_5r,len_t_list);
    double score_2_5l=score_traj(costmap_ros,s_2_5l,len_t_list);
    double score_2_5r=score_traj(costmap_ros,s_2_5r,len_t_list);
    double score_0=score_traj(costmap_ros,s_0,len_t_list);

    multimap<double,double**> score_s_list;
    score_s_list.insert(make_pair(score_10l,s_10l));
    score_s_list.insert(make_pair(score_10r,s_10r));
    score_s_list.insert(make_pair(score_7_5l,s_7_5l));
    score_s_list.insert(make_pair(score_7_5r,s_7_5r));
    score_s_list.insert(make_pair(score_5l,s_5l));
    score_s_list.insert(make_pair(score_5r,s_5r));
    score_s_list.insert(make_pair(score_2_5l,s_2_5l));
    score_s_list.insert(make_pair(score_2_5r,s_2_5r));
    score_s_list.insert(make_pair(score_0,s_0));

    double score_min=score_s_list.begin()->first;
    double** traj_select;
    typedef std::multimap<double,double**>::iterator multimap_iterator;

    if(score_min==0.0 && score_s_list.count(score_min)<9) {
        ROS_INFO("more than 1 traj's score is 0");
        pair<multimap_iterator,multimap_iterator> it =score_s_list.equal_range(score_min);
        while (it.first!=it.second)
        {
            it.first=score_s_list.erase(it.first);
        }
        traj_select=score_s_list.begin()->second;
    }
    else if(score_s_list.count(score_min)>=9){
        ROS_INFO("all traj is score 0,skip this time,use 0 steering as selected traj");
        traj_select=s_0;
    }
    else{
        ROS_INFO("Min traj found");
        traj_select=score_s_list.begin()->second;
    }

    /*
    if(score_min==0.0 && score_s_list.count(score_min)>1) {
        ROS_INFO("more than 1 traj's score is 0, skip this time,use 0 steering as selected traj");
        traj_select=s_0;
    }
    else{
        ROS_INFO("Min traj found");
        traj_select=score_s_list.begin()->second;
    }
     */
    std::cout<<"10l:"<<score_10l<<"\n"<< "10r:"<<score_10r<<"\n" << "7.5l:"<<score_7_5l<<"\n" << "7.5r:"<<score_7_5r<<"\n"<< "5l:"<<score_5l<<"\n"<< "5r:"<<score_5r<<"\n"<< "2.5l:"<<score_2_5l<<"\n" << "2.5r:"<<score_2_5r<<"\n"<< "0:"<<score_0<<std::endl;

    traj_vis(s_10l,s_10r,s_7_5l,s_7_5r,s_5l,s_5r,s_2_5l,s_2_5r,s_0,traj_select,len_t_list);
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
    steering_current=0;
}
double score_traj(costmap_2d::Costmap2DROS& costmap_ros,double** s_array,int len_t_list){
    double score=0.0,cost=0.0;
    bool trans_succ=true;
    unsigned int x_map=0,y_map=0;
    costmap_2d::Costmap2D map=*(costmap_ros.getLayeredCostmap()->getCostmap());
    for(int i=0;i<len_t_list;++i){
        trans_succ=map.worldToMap(s_array[i][0],s_array[i][1],x_map,y_map);
        if(trans_succ) {
            //std::cout << static_cast<double>(costmap_ros.getLayeredCostmap()->getCostmap()->getCost(x_map, y_map)) << std::endl;
            //std::cout<<x_map<<"\n"<<y_map<<std::endl;
            cost=static_cast<double>(map.getCost(x_map, y_map));
            ROS_INFO("cost: %f",cost);
            score += cost;
        }
        else{
            //std::cout<<"trans from world to map failed"<<std::endl;
        }
    }
    return score;
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
        lattice_planner(lcr);
        r.sleep();
    }

    return (0);
}