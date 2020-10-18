#ifndef VLP16_IMPORTER_h
#define VLP16_IMPORTER_h

#include "lidar_package/CONSTANTS.h"
#include "lidar_package/classes/Odometry.h"
#include "lidar_package/scan_struct.h"

// For DEBUG START
#include <iostream>
// For DEBUG END

#include <memory>
#include "ros/ros.h"
#include "velodyne_msgs/VelodynePacket.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// Constants
#define DIST_TO_METERS 0.2/100.0
#define METERS_TO_DIST 100.0/0.2
#define TIME_BETWEEN_PACKETS 0.001327 // [s]

// Point filters
#define REMOVE_UPWARDS_DIRECTIONS true
#define VALID_LIDAR_ANGLE 100.0 // [degrees] 180 being all points because +/- 180 degrees
#define VALID_MAX_DISTANCE 40.0 // [m]
#define VALID_MIN_DISTANCE 1.0 // [m]
#define VALID_ABS_Y_MAX_DISTANCE 15.0 // [m]
#define VALID_Z_MIN_DISTANCE -5.0 // [m]

#define CIRCLE_SIDES_DISTANCE 13.0
#define CIRCLE_SIDES_RADIUS 5.0
#define CIRCLE_SIDES_RADIUS_SQR CIRCLE_SIDES_RADIUS*CIRCLE_SIDES_RADIUS

// True ground definitions for ground detection
#define TRUE_GROUND_HIGH -1.1 // [m]
#define TRUE_GROUND_TOLERANCE 0.2 // [m]
#define TRUE_GROUND_HIGH_MIN TRUE_GROUND_HIGH - TRUE_GROUND_TOLERANCE
#define TRUE_GROUND_HIGH_MAX TRUE_GROUND_HIGH + TRUE_GROUND_TOLERANCE
#define TRUE_GROUND_MAX_DISTANCE 8.0 // [m]

// Horizontal neighbors
#define MAX_DISTANCE_TO_HORIZONTAL_NEIGHBORS (uint32_t) (0.2 * METERS_TO_DIST) // [m]
// Vertical neighbors
#define MAX_HEIGHT_CHANGE 0.1 // [m]
#define MAX_SLOPE_VERTICAL_NEIGHBORS 5.0 * DEGREES_TO_RADIANS // [degrees]
#define MAX_SLOPE_CHANGE_VERTICAL_NEIGHBORS 3.0 * DEGREES_TO_RADIANS // [degrees]

// Roughness calculation
#define CALCULATE_ROUGHNESS false
#define ROUGHNESS_WINDOW_SIZE 7

extern pcl::PointXYZ lidar2Point(uint16_t R, float cos_omega, float sin_omega, float cos_alpha, float sin_alpha);
extern float Distance(pcl::PointXYZ * p);
bool preFilter(uint16_t channel_index, uint16_t distance, float azimuth);
bool postFilter(pcl::PointXYZ * p);
extern void importPoints(Package * point_package, pcl::PointCloud<pcl::PointXYZ>::Ptr points, odo_data * odo = 0, bool applyFilter = true);
extern void importPackage(const velodyne_msgs::VelodyneScan &pkt, pcl::PointCloud<pcl::PointXYZ>::Ptr points, Odometry * odo_object = 0, ros::Time * lidar_timestamp = 0, bool applyFilter = true);
extern void importSimPoints(const sensor_msgs::PointCloud2 &pkt, pcl::PointCloud<pcl::PointXYZ>::Ptr points, Odometry * odo_object = 0, ros::Time * lidar_timestamp = 0, bool applyFilter= true);

enum NodeType{
    UNCHECKED                   = 1 << 0,
    GROUND                      = 1 << 1,
    SLOPE_AVAILIBLE             = 1 << 2,
    SLOPE_CONFIRMED             = 1 << 3,
    SLOPE_CHANGE_AVAILIBLE      = 1 << 4,
    SLOPE_CHANGE_CONFIRMED      = 1 << 5,
    NEIGBOR_APPROVAL_AVAILIBLE  = 1 << 6,
    NEIGBOR_APPROVAL            = 1 << 7 
};

class Node
{
        typedef std::weak_ptr<Node> neighbor;
        neighbor up;
        neighbor down;
        neighbor left;
        neighbor right;
    public:
        typedef std::shared_ptr<Node> ptr;
        pcl::PointXYZ p;
        uint32_t id;
        uint8_t type;
        uint16_t distance;
        uint16_t azimuth;
        uint8_t channel;
        float roughness;
        float vertical_slope;
        // Constructors
        Node();
        Node(uint32_t in_id);
        Node(uint32_t * id_ptr);
        ~Node();
        
        // Neighbor handling
            // Getters
        ptr Up() const{
            return up.lock();
        }
        ptr Down() const{
            return down.lock();
        }
        ptr Left() const{
            return left.lock();
        }
        ptr Right() const{
            return right.lock();
        }
            // Is setters
        bool upSet() const{
            return !up.expired();
        };
        bool downSet() const{
            return !down.expired();
        };
        bool leftSet() const{
            return !left.expired();
        };
        bool rightSet() const{
            return !right.expired();
        };
            // Setters
        void setUp(Node::ptr in_up){
            up = in_up;
        };
        void setDown(Node::ptr in_down){
            down = in_down;
        };
        void setLeft(Node::ptr in_left){
            left = in_left;
        };
        void setRight(Node::ptr in_right){
            right = in_right;
        };
            // Clearers
        void clearUp(){
            up.reset();
        }
        void clearDown(){
            down.reset();
        }
        void clearLeft(){
            left.reset();
        }
        void clearRight(){
            right.reset();
        }
        void clear(){
            up.reset();
            down.reset();
            left.reset();
            right.reset();
        }
        // Disconnectors
        void disconnectUp(){
            if(upSet()){
                Up()->clearDown();
                up.reset();
            } 
        }
        void disconnectDown(){
            if(downSet()){
                Down()->clearUp();
                down.reset();
            } 
        }
        void disconnectLeft(){
            if(leftSet()){
                Left()->clearRight();
                left.reset();
            } 
        }
        void disconnectRight(){
            if(rightSet()){
                Right()->clearLeft();
                right.reset();
            } 
        }
        void disconnect(){
            disconnectUp();
            disconnectDown();
            disconnectLeft();
            disconnectRight();
        }
        float sqrDistanceTo(Node::ptr other_node) const;
        float distanceTo(Node::ptr other_node) const;
        void printNode() const;
};
class Graph
{
    public:
        typedef std::shared_ptr<Graph> ptr;
        std::vector<Node::ptr> nodes;
        pcl::PointXYZ origin;
        ros::Time timestamp;
        /*
        Use reset to remove from vector
        Use sharedptr as bool to check is set.
        
        */
        std::vector<Node::ptr> import_frontier;   
        
        Graph();
        ~Graph();

        void connectVertical(Node::ptr top, Node::ptr bottom){
            top->setDown(bottom);
            bottom->setUp(top);
        }
        void connectHorizontal(Node::ptr left, Node::ptr right){
            left->setRight(right);
            right->setLeft(left);
        }
        void addNode(Node::ptr new_node){
            nodes.push_back(new_node);
        }
        void removeNode(size_t i){ // Removes node completely
            nodes.at(i)->disconnect();
            nodes.erase(nodes.begin()+i);    
        }
        void importNodesFromPackage(Package * point_package, odo_data * in_odo = 0, bool applyFilter = true);
        void importNodesFromScan(const velodyne_msgs::VelodyneScan &pkt, Odometry * odo_object = 0, ros::Time * lidar_timestamp = 0, bool applyFilter = true);
        void exportToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo);
        void exportVerticalToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo);
        void exportHorizontalToList(pcl::PointCloud<pcl::PointXYZ>::Ptr pointListFrom, pcl::PointCloud<pcl::PointXYZ>::Ptr pointListTo);
        void exportObstaclePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points = 0) const;
        void exportToPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points);
        void exportToPoints2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points); // Testing
        void exportToPoints3(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points); // Testing

}; 

#endif