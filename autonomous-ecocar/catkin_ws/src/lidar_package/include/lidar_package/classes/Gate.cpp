#include "Gate.h"


Gate::Gate(){
  is_set = false;
  has_timestamp = false;
};
Gate::Gate(pcl::PointXYZ in_center, float in_fit_fraction, float in_fit_deviation, int in_fitted_points){
  center = in_center;
  fit_fraction = in_fit_fraction;
  fit_deviation = in_fit_deviation;
  fitted_points = in_fitted_points;
  is_set = true;
  has_timestamp = false;
};
void Gate::operator+=(Gate& newGate){
    if(this->timestamp < newGate.getTimestamp()) this->timestamp = newGate.getTimestamp(); // Keep newest
    this->center.x += newGate.center.x;
    this->center.y += newGate.center.y;
    this->center.z += newGate.center.z;
    this->fit_fraction += newGate.fit_fraction;
    this->fit_deviation += newGate.fit_deviation;
    this->fitted_points += newGate.fitted_points;
};
Gate Gate::operator/(float divider){
    pcl::PointXYZ p;
    p.x = this->center.x / divider;
    p.y = this->center.y / divider;
    p.z = this->center.z / divider;
    Gate result(p, this->fit_fraction/divider, this->fit_deviation/divider, ((float) this->fitted_points)/divider);
    result.setTimestamp(this->getTimestamp());
    return result;
};
bool Gate::isSet(){
  return is_set;
};
void Gate::setIsSet(bool state){
  is_set = state;
};
void Gate::setTimestamp(ros::Time in_timestamp){
  timestamp = in_timestamp;
  has_timestamp = true;
};
bool Gate::hasTimestamp(){
  return has_timestamp;
};
ros::Time Gate::getTimestamp(){
  return timestamp;
};