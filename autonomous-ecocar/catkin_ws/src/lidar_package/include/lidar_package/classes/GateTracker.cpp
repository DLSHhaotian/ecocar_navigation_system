#include "GateTracker.h"

GateTracker::GateTracker(int in_window_size_gate, int in_window_size_persistance){
    window_size_gate = in_window_size_gate;
    window_size_persistance = in_window_size_persistance;
    // default values
    min_entry_persistance = 0.6;
    min_exit_persistance = 0.2;
    min_fit_fraction = 0.7; // Also set in RANSAC!
    max_deviation = 0.0007; 
    min_fitted_points = 6;
    max_distance = 0.2;
    max_distance_sqr = max_distance * max_distance;
    min_gate_pair_distance = 2.0;
    min_gate_pair_distance_sqr = min_gate_pair_distance * min_gate_pair_distance;
    max_gate_pair_distance = 4.2;
    max_gate_pair_distance_sqr = max_gate_pair_distance * max_gate_pair_distance;

};
void GateTracker::addTrackedGate(Gate& newGate){
    tracked_gates.push_back(std::make_shared<TrackedGate>(window_size_gate, window_size_persistance, newGate));
};

int GateTracker::findMatch(Gate& newGate){
    for (int i = 0; i < tracked_gates.size(); i++)
    {
        if(tracked_gates.at(i)->distanceToSqr(newGate) <= max_distance_sqr) return i;
    }
    return -1;
}
void GateTracker::mergeTrackedGate(Gate& newGate, int index){
    tracked_gates.at(index)->merge(newGate);
};
void GateTracker::operator+=(Gate& newGate)
{
    // Evaluate if good enough
    if(newGate.fit_fraction >= min_fit_fraction && 
    newGate.fit_deviation <= max_deviation && 
    newGate.fit_fraction >= min_fit_fraction &&
    newGate.fitted_points >= min_fitted_points){
        // Search for match
        int index = findMatch(newGate);
        // Add or merge
        if(index == -1){
            addTrackedGate(newGate);
        } else {
            mergeTrackedGate(newGate, index);
        }
    }
};
void GateTracker::updatePersistance()
{
    for (size_t i = 0; i < tracked_gates.size(); i++)
    {
        tracked_gates.at(i)->updatePersistance();
    }
};
void GateTracker::trim()
{
    // Trim empty gates (zero persistance)
    tracked_gates.erase(std::remove_if(
    tracked_gates.begin(), tracked_gates.end(),
    [](TrackedGate::ptr& tracked_gate) { 
        return (tracked_gate->getPersistance() == 0.0); // put your condition here
    }), tracked_gates.end());
};

void GateTracker::updateActivity()
{
    for (size_t i = 0; i < tracked_gates.size(); i++)
    {
        float persistence_temp = tracked_gates.at(i)->getPersistance();
        bool active_state = tracked_gates.at(i)->getActiveState();
        if(active_state){
            if(persistence_temp <= min_exit_persistance)  tracked_gates.at(i)->setActiveState(false);
        }else{
            if(persistence_temp >= min_entry_persistance)  tracked_gates.at(i)->setActiveState(true);
        }
    }
};

void GateTracker::update()
{
    updatePersistance();
    trim();
    updateActivity();
};
void GateTracker::findGatePairs(Cloud_Ptr gate_pair_centers, Cloud_Ptr gate_column_1, Cloud_Ptr gate_column_2)
{   
    bool addColumns = gate_column_1 != nullptr && gate_column_2 ? true : false;
    Cloud_Ptr all_gate_points(new pcl::PointCloud<pcl::PointXYZ>);
    exportGatesToPoints(all_gate_points);
    int length = all_gate_points->size();
    for (int first_gate_index = 0; first_gate_index < length; first_gate_index++)
    {
        for (int second_gate_index = first_gate_index + 1; second_gate_index < length; second_gate_index++)
        {
            float distance_between_gates_sqr = distanceSqr(all_gate_points->at(first_gate_index), all_gate_points->at(second_gate_index));
            if(distance_between_gates_sqr <= max_gate_pair_distance_sqr && 
            distance_between_gates_sqr >= min_gate_pair_distance_sqr)
            {
                gate_pair_centers->push_back(getMiddlePoint(all_gate_points->at(first_gate_index), all_gate_points->at(second_gate_index)));
                if(addColumns)
                {
                    gate_column_1->push_back(all_gate_points->at(first_gate_index));
                    gate_column_2->push_back(all_gate_points->at(second_gate_index));
                }
            }
        }
    }
}

void GateTracker::exportGatesToPoints(Cloud_Ptr cloud)
{
    for (size_t i = 0; i < tracked_gates.size(); i++)
    {
        if(tracked_gates.at(i)->getActiveState()){
            Gate temp = tracked_gates.at(i)->getAvgGate();
            cloud->push_back(temp.center);
        }
    }
};


void GateTracker::set_min_entry_persistance(float value){
    min_entry_persistance = value;
};
void GateTracker::set_min_exit_persistance(float value){
    min_exit_persistance = value;
};
void GateTracker::set_min_fit_fraction(float value){
    min_fit_fraction = value;
};
void GateTracker::set_max_deviation(float value){
    max_deviation = value;
};
void GateTracker::set_min_fitted_points(int value){
    min_fitted_points = value;
};
void GateTracker::set_max_distance(int value){
    max_distance = value;
    max_distance_sqr = max_distance * max_distance;
};
void GateTracker::set_min_gate_pair_distance(int value){
    min_gate_pair_distance = value;
    min_gate_pair_distance_sqr = min_gate_pair_distance * min_gate_pair_distance;
};
void GateTracker::set_max_gate_pair_distance(int value){
    max_gate_pair_distance = value;
    max_gate_pair_distance_sqr = max_gate_pair_distance * max_gate_pair_distance;
};
void GateTracker::printTrackedGates(){
    std::cout << "GATES" << std::endl;
    for (size_t i = 0; i < tracked_gates.size(); i++)
    {
        Gate temp = tracked_gates.at(i)->getAvgGate();
        float temp_persistance = tracked_gates.at(i)->getPersistance();
        bool temp_active_state = tracked_gates.at(i)->getActiveState();
        std::cout << "\tGate " << i << " has x: " << temp.center.x << " y: " << temp.center.y << " z: " << temp.center.z << std::endl;
        std::cout << "\t\tfit_fraction: " << temp.fit_fraction << " and fit_deviation: " << temp.fit_deviation << " and fitted_points: " << temp.fitted_points  << std::endl;
        std::cout << "\t\tPersistance: " << temp_persistance << " and Activity: " << temp_active_state << std::endl;
    }
};
lidar_package::point GateTracker::pointToMsg(pcl::PointXYZ& point){
    lidar_package::point point_msg;
    point_msg.x = point.x;
    point_msg.y = point.y;
    point_msg.z = point.z;
    return point_msg;
};
lidar_package::gate GateTracker::gatePointsToMsg(pcl::PointXYZ& center, pcl::PointXYZ& column_1, pcl::PointXYZ& column_2){
    lidar_package::gate gate_msg;
    gate_msg.center = pointToMsg(center);
    gate_msg.column_1 = pointToMsg(column_1);
    gate_msg.column_2 = pointToMsg(column_2);
    return gate_msg;
};
lidar_package::gates GateTracker::gatesToMsg(float x, float y, float z, float orientation, ros::Time timestamp){
  lidar_package::gates gates_msg;
  Cloud_Ptr gate_points (new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_Ptr gate_column_1 (new pcl::PointCloud<pcl::PointXYZ>);
  Cloud_Ptr gate_column_2 (new pcl::PointCloud<pcl::PointXYZ>);
  findGatePairs(gate_points, gate_column_1, gate_column_2);

  gates_msg.x = x;
  gates_msg.y = y;
  gates_msg.z = z;
  gates_msg.orientation = orientation;
  gates_msg.vector_len = gate_points->size();
  gates_msg.timestamp = timestamp;
  for (size_t i = 0; i < gates_msg.vector_len; i++){
    gates_msg.vector_gates.push_back(gatePointsToMsg(gate_points->at(i), gate_column_1->at(i), gate_column_2->at(i)));
  }
  return gates_msg;
}
// lidar_package::gates GateTracker::allGatesToMsg(float x, float y, float z, float orientation, ros::Time timestamp){
//   lidar_package::gates gates_msg;
//   Cloud_Ptr gate_points (new pcl::PointCloud<pcl::PointXYZ>);
//   exportGatesToPoints(gate_points);

//   gates_msg.x = x;
//   gates_msg.y = y;
//   gates_msg.z = z;
//   gates_msg.orientation = orientation;
//   gates_msg.vector_len = gate_points->size();
//   gates_msg.timestamp = timestamp;
//   for (size_t i = 0; i < gates_msg.vector_len; i++){
//     gates_msg.vector_gates.push_back(pointToMsg(gate_points->at(i)));
//   }
//   return gates_msg;
// }

// lidar_package::gates GateTracker::gatePairCentersToMsg(float x, float y, float z, float orientation, ros::Time timestamp){
//   lidar_package::gates gates_msg;
//   Cloud_Ptr gate_points (new pcl::PointCloud<pcl::PointXYZ>);
//   findGatePairs(gate_points);

//   gates_msg.x = x;
//   gates_msg.y = y;
//   gates_msg.z = z;
//   gates_msg.orientation = orientation;
//   gates_msg.vector_len = gate_points->size();
//   gates_msg.timestamp = timestamp;
//   for (size_t i = 0; i < gates_msg.vector_len; i++){
//     gates_msg.vector_gates.push_back(pointToMsg(gate_points->at(i)));
//   }
//   return gates_msg;
// };
