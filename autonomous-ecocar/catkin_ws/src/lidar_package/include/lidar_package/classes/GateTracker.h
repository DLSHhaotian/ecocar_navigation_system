#pragma once
#include "Point.h"
#include "Gate.h"
#include "TrackedGate.h"
#include <pcl/point_cloud.h>
#include "lidar_package/point.h"
#include "lidar_package/gate.h"
#include "lidar_package/gates.h"
#include <iostream> // Debug

class GateTracker
{
  private:
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_Ptr;
    
    int window_size_gate;
    int window_size_persistance;
    float min_entry_persistance;
    float min_exit_persistance;
    float min_fit_fraction;
    float max_deviation;
    int min_fitted_points;
    float max_distance;
    float max_distance_sqr;
    float min_gate_pair_distance;
    float min_gate_pair_distance_sqr;
    float max_gate_pair_distance;
    float max_gate_pair_distance_sqr;
    std::vector<TrackedGate::ptr> tracked_gates;

  public:
    GateTracker(int window_size_gate, int window_size_persistance);
    void addTrackedGate(Gate& newGate);
    int findMatch(Gate& newGate);
    void mergeTrackedGate(Gate& newGate, int index);
    void operator+=(Gate& newGate);
    void updatePersistance();
    void trim();
    void updateActivity();
    void update();
    void findGatePairs(Cloud_Ptr gate_pair_centers, Cloud_Ptr gate_column_1 = nullptr, Cloud_Ptr gate_column_2 = nullptr);
    void exportGatesToPoints(Cloud_Ptr cloud);
    void set_min_entry_persistance(float value);
    void set_min_exit_persistance(float value);
    void set_min_fit_fraction(float value);
    void set_max_deviation(float value);
    void set_min_fitted_points(int value);
    void set_max_distance(int value);
    void set_min_gate_pair_distance(int value);
    void set_max_gate_pair_distance(int value);
    void printTrackedGates();
    lidar_package::point pointToMsg(pcl::PointXYZ& point);
    lidar_package::gate gatePointsToMsg(pcl::PointXYZ& center, pcl::PointXYZ& column_1, pcl::PointXYZ& column_2);
    lidar_package::gates gatesToMsg(float x, float y, float z, float orientation, ros::Time timestamp);
};