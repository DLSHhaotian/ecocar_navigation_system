#include "Odometry.h"


odo_data * Odometry::getData(int32_t i){
  int32_t index = top_index - i;
  if (index < 0){
    if (log_is_full){
      index = index + log_size;
    }else{
      ROS_ERROR("odo log overflow");
      return 0;
    }
  }
  return &data_log.at(index);
};

odo_data Odometry::interpolateData(odo_data * from_odo, odo_data * to_odo, ros::Time time){
  if(USE_NEAREST_POINT){ // Use nearest data point insted of interpolation
    ros::Duration duration_first = time - from_odo->time_of_receival;
    ros::Duration duration_second = to_odo->time_of_receival - time;
    if(duration_first <= duration_second){
      return * from_odo;
    }else{
      return * to_odo;
    }
  }else{
    odo_data result = * from_odo;
    result.time_of_receival = time;
    ros::Duration duration_fraction = time - from_odo->time_of_receival;
    ros::Duration duration_total = to_odo->time_of_receival - from_odo->time_of_receival;
    if(duration_total <= MIN_DURATION) return result;
    float k = duration_fraction.toSec()/duration_total.toSec();
    result.x_pos += (to_odo->x_pos - from_odo->x_pos)*k;
    result.y_pos += (to_odo->y_pos - from_odo->y_pos)*k;
    result.z_pos += (to_odo->z_pos - from_odo->z_pos)*k;
    result.orientation += (to_odo->orientation - from_odo->orientation)*k;
    result.speed += (to_odo->speed - from_odo->speed)*k;
    result.acceleration += (to_odo->acceleration - from_odo->acceleration)*k;
    cos_orientation = cos(result.orientation); 
    sin_orientation = sin(result.orientation);
    return result;
  }
};

Odometry::Odometry(int32_t in_log_size){
  ros::Time::init();
  // Init current
  x_pos = 0; 
  y_pos = 0; 
  z_pos = 0; 
  orientation = 0; 
  speed = 0; 
  acceleration = 0; 
  cos_orientation = 1; 
  sin_orientation = 0; 
  time_of_receival = ros::Time::now();  

  // Log inits
  // data_log.resize(log_size);
  log_size = in_log_size;
  top_index = 0;
  log_is_full = false;

  odo_data inital_odo_data;
  // Set inital values for log
  inital_odo_data.x_pos = x_pos;
  inital_odo_data.y_pos = y_pos;
  inital_odo_data.z_pos = z_pos;
  inital_odo_data.orientation = orientation;
  inital_odo_data.speed = speed;
  inital_odo_data.acceleration = acceleration;
  inital_odo_data.cos_orientation = cos_orientation;
  inital_odo_data.sin_orientation = sin_orientation;
  inital_odo_data.time_of_receival = time_of_receival;    
  for (size_t i = 0; i < in_log_size; i++)
  {
    data_log.push_back(inital_odo_data);
  }
};

void Odometry::update(float in_x, float in_y, float in_z, float in_orientation, float in_speed, float in_acceleration){
  // Update current
  x_pos = in_x; 
  y_pos = in_y; 
  z_pos = in_z; 
  orientation = in_orientation; 
  speed = in_speed; 
  acceleration = in_acceleration; 
  cos_orientation = cos(in_orientation); 
  sin_orientation = sin(in_orientation); 
  time_of_receival = ros::Time::now(); // TODO: Add delay

  // Update log
  top_index++;
  if(top_index >= log_size){
    top_index = 0;
    log_is_full = true;
  }
  data_log.at(top_index).x_pos = x_pos;
  data_log.at(top_index).y_pos = y_pos;
  data_log.at(top_index).z_pos = z_pos;
  data_log.at(top_index).orientation = orientation;
  data_log.at(top_index).speed = speed;
  data_log.at(top_index).acceleration = acceleration;
  data_log.at(top_index).cos_orientation = cos_orientation;
  data_log.at(top_index).sin_orientation = sin_orientation;
  data_log.at(top_index).time_of_receival = time_of_receival; 

};

void Odometry::transformPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl){
  for (size_t i = 0; i < pcl->size(); i++)   
  { 
    float x = pcl->at(i).x; 
    float y = pcl->at(i).y; 
    float z = pcl->at(i).z; 

    pcl->at(i).x = x*cos_orientation - y*sin_orientation + x_pos; 
    pcl->at(i).y = x*sin_orientation + y*cos_orientation + y_pos; 
    pcl->at(i).z = z; 
  }
};

void Odometry::transformClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > * pcls){
  for (size_t i = 0; i < pcls->size(); i++)
  {
    Odometry::transformPoints(pcls->at(i));
  }
};

odo_data Odometry::getInterpolatedData(ros::Time time){
  int32_t max_steps_back = log_is_full ? log_size : top_index;
  odo_data * from_odo;
  odo_data * to_odo;
  // If required time is newer than data
  from_odo = getData(0);
  if(from_odo->time_of_receival < time) return * from_odo;
  // If required time is within data
  for (size_t i = 1; i < max_steps_back; i++)
  {
    to_odo = from_odo;
    from_odo = getData(i);
    if(from_odo->time_of_receival < time && time <= to_odo->time_of_receival){
      return interpolateData(from_odo, to_odo, time);
    }
  }
  // If required time is before data
  if(time <= from_odo->time_of_receival) return * from_odo;
};

void Odometry::printLog(){
  if(log_is_full){
    for (uint32_t i = 0; i < log_size; i++)
    {
      odo_data * current = getData(i);
      ROS_INFO("(At: %f)\tx: %f,\ty: %f,\tz: %f", current->time_of_receival.toSec(), current->x_pos, current->y_pos, current->z_pos);
    }
  }else{
    for (uint32_t i = 0; i < top_index; i++)
    {
      odo_data * current = getData(i);
      ROS_INFO("(At: %f)\tx: %f,\ty: %f,\tz: %f", current->time_of_receival.toSec(), current->x_pos, current->y_pos, current->z_pos);
    }
  };
};
