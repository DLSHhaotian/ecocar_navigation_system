#include "ros/ros.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
// Messages
#include "auto_navi/driveMsg.h"
#include "dynamo_msgs/TeensyRead.h"
#include "dynamo_msgs/TeensyWrite.h"
#include "geometry_msgs/Point.h"
#include "lidar_package/obst.h"
#include "lidar_package/obsts.h"
#include "lidar_package/point.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <sstream>

// Relevant for detecting barriers
double OBST_MAX_DIST =
    15.0;                // The maximum distance an obstacle can be from the car
double SCAN_ANGLE = 0.5; /*
     * This correlates to the SCAN_ANGLE that we look to the
 * left and right.
     * if SCAN_ANGLE < 1 the SCAN_ANGLE gets more stump and is
 * SCAN_ANGLE > 1 the
     * SCAN_ANGLE gets more sharp
     * SCAN_ANGLE = 1 results in a 90 degree SCAN_ANGLE. The
 * number is the slope of
     * the partition.
     * To get in degrees/radians: 90 - atan(SCAN_ANGLE)
     */
int BARRIER_LIMIT = 10;  // Minimum amount of obstacles required to be percieved
                         // as a barrier on either side
double LR_RATIO_LIMIT = 3.0; // std: 3.0

double DISPLACEMENT_DISTANCE_Y = 3.5; // Diplacement of linear regression. i.e.
                                      // how far from the barrier should we go?
double DISPLACEMENT_DISTANCE_X =
    4.0; // How far in front of the car should the dp be located
double MAX_REGRESSION_DIST = 20.0; // the maximum distance points can be from
                                   // the car to be included in regressioon
                                   // (along x axis)
double MIN_REGRESSION_DIST = 0.0; // the minimum distance points can be from the
                                  // car to be included in regressioon (along x
                                  // axis)
double STEPSIZE = 1.0; // Stepsize has to be greater than the distance between
                       // points. If 0, returns all points.

struct obstpoint {
  geometry_msgs::Point point;
  int id;
};

/*  GLOBAL VARIABLES */
ros::Publisher drivepointPub;
ros::Publisher barriersVisiblePub;
ros::Publisher marker_pub;

std_msgs::String barriersVisible;
double odox;     // odometry x in world coordinates
double odoy;     // odometry y in world coordinates
double odotheta; // odometry orientation

int leftcount = 0; // to keep track of how many barriers are found on each side
int rightcount = 0;
// Distance from barrier to car in [m]

int leftid, rightid; // IDs of left and right barrier
int closestbarrier;
double closestobstacledistanceleft;
double closestobstacledistanceright;
bool voronoiused = true;
enum Situation { SIT_NONE, SIT_LEFT, SIT_RIGHT, SIT_BOTH };

volatile int id = 0;

// Function prototypes
void rviz_pointsused(std::vector<obstpoint> barrierSide, std::string ns,
                     float r, float g, float b);
void rvizShowRegression(std::vector<double>);

bool xsort(
    const obstpoint &a,
    const obstpoint &b) // utility used to sort obstacle points due to x values.
{
  return a.point.x < b.point.x;
}

std::vector<obstpoint> groupbarrier(std::vector<obstpoint> localobstacles) {
  obstpoint obstaclefound;

  double step = 0.0;
  bool found = false;

  double innermost = 9999.9;

  std::vector<obstpoint> barrierSide;

  if (barriersVisible.data == "Right") {
    innermost = -9999.9;
  }

  for (int i = 0; i < (localobstacles.size() - 1); i++) {
    // only look on barriers for the first X meters.
    double distanceFromCar = sqrt(pow(localobstacles[i].point.x, 2.0) +
                                  pow(localobstacles[i].point.y, 2.0));
    if (distanceFromCar > MAX_REGRESSION_DIST ||
        distanceFromCar < MIN_REGRESSION_DIST ||
        localobstacles[i].point.x < step) {
      continue;
    } else if (barriersVisible.data == "Left" &&
               localobstacles[i].id == leftid) {

      bool isInnerLeft = localobstacles[i].point.y < innermost;
      bool inCurrentStep = localobstacles[i].point.x >= step &&
                           localobstacles[i].point.x < (step + STEPSIZE);

      if (isInnerLeft && inCurrentStep) {
        innermost = localobstacles[i].point.y;
        obstaclefound = localobstacles[i];
        found = true;
      }

      if (localobstacles[i + 1].point.x > step + STEPSIZE) {
        if (found) {
          step += STEPSIZE;
          barrierSide.push_back(obstaclefound);
          found = false;
          innermost = 9999.9;
        } else {
          step = localobstacles[i + 1].point.x;
          if (localobstacles[i + 1].point.x - localobstacles[i].point.x >
              STEPSIZE) {
            barrierSide.push_back(localobstacles[i]);
          }
        }
      }
    } else if (barriersVisible.data == "Right" &&
               localobstacles[i].id == rightid) {

      bool isInnerRight = localobstacles[i].point.y > innermost;
      bool inCurrentStep = localobstacles[i].point.x > step &&
                           localobstacles[i].point.x < (step + STEPSIZE);

      if (isInnerRight && inCurrentStep) {
        innermost = localobstacles[i].point.y;
        obstaclefound = localobstacles[i];
        found = true;
      }

      if (localobstacles[i + 1].point.x > step + STEPSIZE) {
        if (found) {
          step += STEPSIZE;
          barrierSide.push_back(obstaclefound);
          found = false;
          innermost = -9999.9;
        } else {
          step = localobstacles[i + 1].point.x;
          if (localobstacles[i + 1].point.x - localobstacles[i].point.x >
              STEPSIZE) {
            barrierSide.push_back(localobstacles[i]);
          }
        }
      }
    }
  }

  rviz_pointsused(barrierSide, "FB:Side barrier for regression", 0.0, 0.0, 1.0);

  return barrierSide;
}

std::vector<double>
linreg(std::vector<obstpoint> obst) { // Linear regression using least squares
                                      // method using an array of obstpoints

  std::vector<double> b0b1;
  double xSum = 0;
  double ySum = 0;
  double xMean;
  double yMean;
  double num = 0;
  double den = 0;

  for (int i = 0; i < obst.size(); i++) {
    xSum += obst[i].point.x;
    ySum += obst[i].point.y;
  }

  xMean = xSum / obst.size();
  yMean = ySum / obst.size();

  for (int i = 0; i < obst.size(); i++) {
    num += (obst[i].point.x - xMean) * (obst[i].point.y - yMean);
    den += pow((obst[i].point.x - xMean), 2);
  }
  double b1 = num / den;          // Slope
  double b0 = yMean - b1 * xMean; // Intersect

  b0b1.push_back(b0);
  b0b1.push_back(b1);

  // Statistics, could be removed for performance
  double SStot = 0, SSres = 0;
  for (int i = 0; i < obst.size(); i++) {
    SStot += pow((obst[i].point.y - yMean), 2);
    SSres += pow((obst[i].point.y - (b0 + obst[i].point.x * b1)), 2);
  }
  double rsquared = 1 - (SSres / SStot);
  // ROS_INFO("R^2 = %f", rsquared);

  return b0b1;
}

auto_navi::driveMsg followbarrier(std::vector<obstpoint> localobstacles) {

  auto_navi::driveMsg drivepoint;

  std::vector<obstpoint> barrierSide;

  barrierSide = groupbarrier(localobstacles);

  /*  Linear regression using least squares method  */
  std::vector<double> b0b1;
  b0b1 = linreg(
      barrierSide); // returns slope and intersect of the linear regression

  rvizShowRegression(b0b1);

  double b0 = b0b1[0];
  double b1 = b0b1[1];

  double drivex = DISPLACEMENT_DISTANCE_X;
  double drivey = b0 + b1 * drivex;
  double drivetheta = atan(b1); // The inverse tangens of the slope is theta.
  double unitx = 1.0 / (sqrt((b1 * b1) + (1 * 1)));
  double unity = b1 / (sqrt((b1 * b1) + (1 * 1))); // slope
  double temp = unitx;

  // "hat"/orthogonal vector for the unitvector.
  if (barriersVisible.data == "Left") {
    unitx = unity;
    unity = -temp;
  } else if (barriersVisible.data == "Right") {
    unitx = -unity;
    unity = temp;
  }

  // Move drivepoint in the orthogonal direction.
  drivex = drivex + unitx * DISPLACEMENT_DISTANCE_Y;
  drivey = drivey + unity * DISPLACEMENT_DISTANCE_Y;

  /* CHANGE DRIVEPOINT BACK TO GLOBAL COORDINATES */
  double globaldrivex = cos(odotheta) * drivex - sin(odotheta) * drivey + odox;
  double globaldrivey = sin(odotheta) * drivex + cos(odotheta) * drivey + odoy;
  double globaldrivetheta = drivetheta + odotheta;

  // Assign drivepoint to return value
  drivepoint.header.stamp = ros::Time::now();
  drivepoint.header.frame_id = "follow_barrier";
  drivepoint.x = globaldrivex;
  drivepoint.y = globaldrivey;
  drivepoint.theta = globaldrivetheta;
  // ROS_INFO("Drive point: [%f, %f, %f]", globaldrivex, globaldrivey,
  //         globaldrivetheta);
  return drivepoint;
}

auto_navi::driveMsg drivestraight() {
  auto_navi::driveMsg drivepoint;

  double localx = 4;
  double localy = 0;
  double localtheta = 0;

  drivepoint.x = cos(odotheta) * localx - sin(odotheta) * localy + odox;
  drivepoint.y = sin(odotheta) * localx + cos(odotheta) * localy + odoy;
  drivepoint.theta = localtheta + odotheta;
  return drivepoint;
}

std::vector<obstpoint>
sortobstacles(const lidar_package::obsts::ConstPtr &msg) {
  std::vector<obstpoint> obstacles; // Return vector of obstacles
  std::vector<obstpoint>
      drawobstacles; // Vector of obstacles inside counted area
  double laserx;     // msg->vector_obst.vector_point.x
  double lasery;     // msg->vector_obst.vector_point.y
  double localx;     // laserx in local coordinates
  double localy;     // lasery in local coordinates
  obstpoint point;
  geometry_msgs::Point xy;

  // Global variables to be assigned in this function
  closestobstacledistanceleft = OBST_MAX_DIST;
  closestobstacledistanceright = OBST_MAX_DIST;
  leftid = 0;
  rightid = 0;
  leftcount = 0;
  rightcount = 0;

  for (int i = 0; i < msg->vector_len;
       i++) { // for each obstacle (i.e. group of points that make an obstacle)
    for (int j = 0; j < msg->vector_obst[i].vector_len; j++) {
      laserx = msg->vector_obst[i].vector_point[j].x;
      lasery = msg->vector_obst[i].vector_point[j].y;
      // matrix transformation from world coordinates to local coordinates
      localx = (cos(-odotheta) * (laserx - odox)) -
               (sin(-odotheta) * (lasery - odoy));
      localy = (sin(-odotheta) * (laserx - odox)) +
               (cos(-odotheta) * (lasery - odoy));

      xy.x = localx;
      xy.y = localy;
      point.point = xy;
      double distance = sqrt(pow(localx, 2) + pow(localy, 2));

      // Find closest barrier id on either side.
      if (distance > OBST_MAX_DIST) {
        /* Do not look farther than OBST_MAX_DIST meters to determine if barrier
         * is on left or right side.*/
      } else if ((localy >= 0) && (fabs(localx) * SCAN_ANGLE < fabs(localy)) &&
                 localx > 0) {
        // leftcount++;
        drawobstacles.push_back(point);
        // check id is the closest
        if (distance < closestobstacledistanceleft) {
          leftid = msg->vector_obst[i].id;
          closestobstacledistanceleft = distance;
          closestbarrier = leftid;
        }
      } else if ((localy < 0) && (fabs(localx) * SCAN_ANGLE < fabs(localy)) &&
                 localx > 0) {
        // rightcount++;
        drawobstacles.push_back(point);
        // Check id is the closest
        if (distance < closestobstacledistanceright) {
          rightid = msg->vector_obst[i].id;
          closestobstacledistanceright = distance;
          closestbarrier = rightid;
        }
      }
      point.id = msg->vector_obst[i].id;
      obstacles.push_back(point);
    }
  }

  for (int i = 0; i < msg->vector_len; i++) {
    if (msg->vector_obst[i].id == leftid) {
      leftcount = msg->vector_obst[i].vector_len;
    }
    if (msg->vector_obst[i].id == rightid) {
      rightcount = msg->vector_obst[i].vector_len;
    }
  }
  if (leftcount < BARRIER_LIMIT) {
    closestobstacledistanceleft = 0;
  }
  if (rightcount < BARRIER_LIMIT) {
    closestobstacledistanceright = 0;
  }
  rviz_pointsused(drawobstacles, "FB:barrier_counted", 0.75, 0.0, 0.75);

  sort(obstacles.begin(), obstacles.end(), xsort);

  return obstacles;
}

void rviz_pointsused(std::vector<obstpoint> barrierSide, std::string ns,
                     float r, float g, float b) {
  std::vector<obstpoint> drawBarrier = barrierSide;

  visualization_msgs::Marker marker;
  ros::Duration sixtyhundred_sec(600.0);
  marker.header.frame_id = "visualization_frame"; //"base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 1;
  uint32_t shape = visualization_msgs::Marker::POINTS;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0; // different from 0, unless you want it to be invisible.
  marker.lifetime =
      sixtyhundred_sec; // ros::Duration means it will never auto-delete.

  for (int i = 0; i < barrierSide.size(); i++) {
    drawBarrier[i].point.x = cos(odotheta) * barrierSide[i].point.x -
                             sin(odotheta) * barrierSide[i].point.y + odox;
    drawBarrier[i].point.y = sin(odotheta) * barrierSide[i].point.x +
                             cos(odotheta) * barrierSide[i].point.y + odoy;
    marker.points.push_back(drawBarrier[i].point);
  }

  marker_pub.publish(marker);
}
void rviz_currentlocation(const lidar_package::obsts::ConstPtr &msg) {

  // This rvizzer draws the global location of the car and gives it a color
  // according to barriersVisible.data
  visualization_msgs::Marker marker;
  ros::Duration sixtyhundred_sec(600.0);
  marker.header.frame_id = "visualization_frame"; //"base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "FB:Position and status";
  id++;
  marker.id = id;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.
  marker.pose.position.x = msg->x;
  marker.pose.position.y = msg->y;
  marker.pose.position.z = -1.0;
  geometry_msgs::Point p;
  p.x = 0; // msg->x;
  p.y = 0; // msg->y;
  marker.points.push_back(p);

  tf2::Quaternion tempquat;
  // Set the orientation of the point
  tempquat.setRPY(0, 0, msg->orientation);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(tempquat);
  marker.pose.orientation = quat_msg;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  if (barriersVisible.data == "Left") {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  } else if (barriersVisible.data == "Right") {
    marker.color.r = 0.2f;
    marker.color.g = 0.2f;
    marker.color.b = 1.0f;
  } else if (barriersVisible.data == "Both") {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else {
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }
  marker.color.a = 1.0; // different from 0, unless you want it to be invisible.
  marker.lifetime =
      sixtyhundred_sec; // ros::Duration means it will never auto-delete.

  marker_pub.publish(marker);
}

/*! Publishes the car's location as a car shaped marker for Rviz.
*   The mesh for the marker doesn't always points to the right place at the pc.
*/
void rviz_cararrow(const lidar_package::obsts::ConstPtr &msg) {

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "ecocar";

  // This rvizzer draws the car as an arrow
  visualization_msgs::Marker marker;
  marker.header.frame_id = "visualization_frame"; //"base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "FB: Car arrow";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource =
      "package://ecocar_description/meshes/ecocar_texture2.dae";
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.
  marker.pose.position.x = msg->x;
  marker.pose.position.y = msg->y;
  marker.pose.position.z = -1.0;
  // Pose of frame
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = msg->z;

  tf2::Quaternion tempquat;
  // Set the orientation of the point
  tempquat.setRPY(0, 0, msg->orientation - 3.14159 / 2.0);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(tempquat);
  marker.pose.orientation = quat_msg;
  // Rotation of ecocarframe
  tempquat.setRPY(0, 0, msg->orientation);
  transformStamped.transform.rotation.x = tempquat.x();
  transformStamped.transform.rotation.y = tempquat.y();
  transformStamped.transform.rotation.z = tempquat.z();
  transformStamped.transform.rotation.w = tempquat.w();

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.6350f;
  marker.color.g = 0.0780f;
  marker.color.b = 0.1840f;

  br.sendTransform(transformStamped);
  marker_pub.publish(marker);
}

// Accepts the regression estimate and publishes a visual estimate to an rviz
// topic.
// Line is between two points far away in some directions.
void rvizShowRegression(std::vector<double> b0b1) {
  // Local coords line
  double b0 = b0b1[0];
  double b1 = b0b1[1];
  visualization_msgs::Marker regression_marker;

  regression_marker.header.frame_id = "visualization_frame";
  regression_marker.header.stamp = ros::Time::now();
  regression_marker.ns = "FB: Regression Line";
  regression_marker.id = 0;
  regression_marker.type = visualization_msgs::Marker::LINE_STRIP;
  regression_marker.action = visualization_msgs::Marker::ADD;
  regression_marker.pose.orientation.w = 1;

  // Width of line
  regression_marker.scale.x = 0.03;
  // Color of line
  regression_marker.color.a = 1.0;
  regression_marker.color.r = 1.0;

  // Points
  geometry_msgs::Point p0, p1, p0local, p1local;
  p0local.x = -20.0; // Arbitrary
  p0local.y = b0 + b1 * p0local.x;
  p1local.x = 20.0; // Arbitraty
  p1local.y = b0 + b1 * p1local.x;

  // Convert points to global coords
  p0.x = cos(odotheta) * p0local.x - sin(odotheta) * p0local.y + odox;
  p0.y = sin(odotheta) * p0local.x + cos(odotheta) * p0local.y + odoy;
  p0.z = 0;
  p1.x = cos(odotheta) * p1local.x - sin(odotheta) * p1local.y + odox;
  p1.y = sin(odotheta) * p1local.x + cos(odotheta) * p1local.y + odoy;
  p1.z = 0;
  // Pushback
  regression_marker.points.push_back(p0);
  regression_marker.points.push_back(p1);

  marker_pub.publish(regression_marker);
}

/*! The main part of the followbarrier code.
*   Recieves the LiDAR data and runs the appropiate functions for counting
* barriers
*   and determining the driving strategy to use.
*/
void obstacle_hullsCallback(const lidar_package::obsts::ConstPtr &msg) {
  ros::Time starttime = ros::Time::now();
  auto_navi::driveMsg dp;
  std::vector<obstpoint> localobstacles;

  odox = msg->x;               // odometry x
  odoy = msg->y;               // odometery y
  odotheta = msg->orientation; // odometry theta
  double lrDistRatio;
  localobstacles = sortobstacles(msg);
  /**
  * sortobstacles(msg) allocates all obstacle points in an array of obstpoints,
  * sorts them and issues them ids wether they are
  * on the left(1) or right(2) side of the car. Then returns the array.
  **/
  // ROS_INFO("leftc %d, rightc %d", leftcount, rightcount);
  if (closestobstacledistanceright == 0) {
    lrDistRatio = std::numeric_limits<double>::infinity();
  } else {
    lrDistRatio = closestobstacledistanceleft / closestobstacledistanceright;
  }
  // ROS_INFO("Ldist, Rdist, Ratio [%f, %f, %f]", closestobstacledistanceleft,
  //         closestobstacledistanceright, lrDistRatio);

  Situation Situation;
  if (!(rightcount | leftcount)) { // if both are zero
    Situation = SIT_NONE;
  } else if (lrDistRatio > LR_RATIO_LIMIT) {
    Situation = SIT_LEFT;
  } else if (lrDistRatio <= 1.0 / LR_RATIO_LIMIT) {
    Situation = SIT_RIGHT;
  } else {
    Situation = SIT_BOTH;
  }

  if (Situation == SIT_BOTH) { /* BOTH BARRIERS VISIBLE */
    //} else if (false) {
    barriersVisible.data = "Both";
    ROS_INFO("barriersVisible.data: Both");
    voronoiused = true;

  } else if (Situation == SIT_LEFT) { /* LEFT BARRIER VISIBLE */
    //} else if (closestbarrier == leftid) {
    ROS_INFO("barriersVisible.data: Left");
    barriersVisible.data = "Left";
    dp = followbarrier(localobstacles);
    // Message flair
    dp.header.stamp = starttime;
    dp.header.frame_id = "follow_barrier";
    drivepointPub.publish(dp);
    voronoiused = false;

  } else if (Situation == SIT_RIGHT) { /* RIGHT BARRIER VISIBLE */
    ROS_INFO("barriersVisible.data: Right");
    //} else if (closestbarrier == rightid) {
    barriersVisible.data = "Right";
    dp = followbarrier(localobstacles);
    // Message flair
    dp.header.stamp = starttime;
    dp.header.frame_id = "follow_barrier";
    drivepointPub.publish(dp);
    voronoiused = false;

  } else { /* See nothing? Just drive straight */
    ROS_INFO("barriersVisible.data: None");
    barriersVisible.data = "None";
    dp = drivestraight();
    // Message flair
    dp.header.stamp = starttime;
    dp.header.frame_id = "follow_barrier";
    drivepointPub.publish(dp);
    voronoiused = false;
  }

  barriersVisiblePub.publish(barriersVisible);
  // ROS_INFO("Runtime: %f", (ros::Time::now() - starttime).toSec());
  rviz_currentlocation(msg);
  rviz_cararrow(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_barrier");
  ros::NodeHandle n("~");

  // Variables, can be changed from the command line. "rosrun auto_navi follow
  // barrier _VAR1_NAME:=VALUE _VAR2_NAME:=2" etc.
  n.param<double>(
      "DISPLACEMENT_DISTANCE_Y", DISPLACEMENT_DISTANCE_Y,
      DISPLACEMENT_DISTANCE_Y); // so we can pass arguments when calling rosrun
  n.param<double>(
      "DISPLACEMENT_DISTANCE_X", DISPLACEMENT_DISTANCE_X,
      DISPLACEMENT_DISTANCE_X); // so we can pass arguments when calling rosrun
  n.param<double>("OBST_MAX_DIST", OBST_MAX_DIST, OBST_MAX_DIST);
  n.param<double>("MAX_REGRESSION_DIST", MAX_REGRESSION_DIST,
                  MAX_REGRESSION_DIST);
  n.param<double>("SCAN_ANGLE", SCAN_ANGLE, SCAN_ANGLE);
  n.param<int>("BARRIER_LIMIT", BARRIER_LIMIT, BARRIER_LIMIT);
  n.param<double>("STEPSIZE", STEPSIZE, STEPSIZE);
  n.param<double>("MIN_REGRESSION_DIST", MIN_REGRESSION_DIST,
                  MIN_REGRESSION_DIST);
  n.param<double>("LR_RATIO_LIMIT", LR_RATIO_LIMIT, LR_RATIO_LIMIT);

  // Subscribers
  ros::Subscriber sub =
      n.subscribe("/obstacle_hulls", 100, obstacle_hullsCallback);

  // Publishers
  drivepointPub = n.advertise<auto_navi::driveMsg>("/drive_points", 1);
  barriersVisiblePub = n.advertise<std_msgs::String>("/barriers_visible", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("/arrow_marker", 0);

  float loopRate = 20; // loop rate in [Hz]
  ros::Rate r(loopRate);
  int count = 0;
  barriersVisible.data = "None";
  ROS_INFO("Starting main loop.");
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();

    ++count;
  }
  return 0;
}
