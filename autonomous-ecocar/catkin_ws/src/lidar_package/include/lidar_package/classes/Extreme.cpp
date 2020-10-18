#include "Extreme.h"

Extreme::Extreme(){
    is_set = false;
};
Extreme::Extreme(
    float input_x_min, float input_x_max, 
    float input_y_min, float input_y_max, 
    float input_z_min, float input_z_max
    ){
    x_min = input_x_min;
    x_max = input_x_max;
    y_min = input_y_min;
    y_max = input_y_max;
    z_min = input_z_min;
    z_max = input_z_max;
    is_set = true;
};
void Extreme::clear(){
    is_set = false;
}
void Extreme::makeExtreme(pcl::PointCloud<pcl::PointXYZ>::Ptr points){
    if(points->size() > 0){
        pcl::PointXYZ initial_point = points->front();

        x_min = initial_point.x;
        x_max = initial_point.x;
        y_min = initial_point.y;
        y_max = initial_point.y;
        z_min = initial_point.z;
        z_max = initial_point.z;
        is_set = true;

        for (size_t i = 1; i < points->size(); i++)
        {
        update(&points->at(i));
        }
    }
};
bool Extreme::isSet(){
    return is_set;
}
bool Extreme::isNearXY(pcl::PointXYZ * p, float dist){
    if(
        p->x >= x_min - dist && p->x <= x_max + dist &&
        p->y >= y_min - dist && p->y <= y_max + dist
        ) return true;
    else return false;
};
pcl::PointCloud<pcl::PointXYZ>::Ptr Extreme::filterNearXY(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float dist){
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t point_index = 0; point_index < points->size(); point_index++)
    {
        if(isNearXY(&(points->at(point_index)), dist)) filtered_points->push_back(points->at(point_index));
    }
    return filtered_points;
};
bool Extreme::isNear(pcl::PointXYZ * p, float dist){
    if(
        p->x >= x_min - dist && p->x <= x_max + dist &&
        p->y >= y_min - dist && p->y <= y_max + dist &&
        p->z >= z_min - dist && p->z <= z_max + dist
        ) return true;
    else return false;
};
bool Extreme::isNear(Extreme * other_extreme, float dist){
    if(
        x_max + dist < other_extreme->x_min || x_min - dist > other_extreme->x_min ||
        y_max + dist < other_extreme->y_min || y_min - dist > other_extreme->y_min ||
        z_max + dist < other_extreme->z_min || z_min - dist > other_extreme->z_min
        ) return false;
    else return true;
};
void Extreme::update(pcl::PointXYZ * p){
    if(!is_set){
        ROS_WARN("Extreme not initalized!!!");
    }else{
        if(p->x < x_min) x_min = p->x;
        else if(p->x > x_max) x_max = p->x;
        if(p->y < y_min) y_min = p->y;
        else if(p->y > y_max) y_max = p->y;
        if(p->z < z_min) z_min = p->z;
        else if(p->z > z_max) z_max = p->z;
    }
};
void Extreme::merge(Extreme * other_extreme){
    if(is_set){
        if(other_extreme->x_min < x_min) x_min = other_extreme->x_min;
        if(other_extreme->x_max > x_max) x_max = other_extreme->x_max;
        if(other_extreme->y_min < y_min) y_min = other_extreme->y_min;
        if(other_extreme->y_max > y_max) y_max = other_extreme->y_max;
        if(other_extreme->z_min < z_min) z_min = other_extreme->z_min;
        if(other_extreme->z_max > z_max) z_max = other_extreme->z_max;
    }else{
        x_min = other_extreme->x_min;
        x_max = other_extreme->x_max;
        y_min = other_extreme->y_min;
        y_max = other_extreme->y_max;
        z_min = other_extreme->z_min;
        z_max = other_extreme->z_max;
        is_set = true;
    }
};
void Extreme::printExtreme(){
    ROS_INFO("x(%.2f, %.2f), y(%.2f, %.2f), z(%.2f, %.2f)", x_min, x_max, y_min, y_max, z_min, z_max);
};
