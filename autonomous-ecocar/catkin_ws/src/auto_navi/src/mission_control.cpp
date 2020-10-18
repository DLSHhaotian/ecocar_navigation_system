#include "ros/ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// Messages
#include "auto_navi/driveMsg.h"
#include "dynamo_msgs/BrakeStepper.h"
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
// Uh, is it needed?
#include <sstream>

/**
 * This node determines what driving strategy is currently in use.
 *
 * It recieves a judgement from the follow_barrier node, on how many barriers is
 * usable for navigation, {0, 1, 2}.
 * If it sees no barriers for 10 seconds, it brakes. (Not implemented as of
 * 19-04-29)
 * Outputs the driving strategy to the motor node, as a drive point
 */

#define PUBLISH_TO_RVIZ true

// Global variables
auto_navi::driveMsg follow_barrier_dp, voronoi_dp, parking_dp, gate_dp;
ros::Time last_not_none_time, last_gate_time;
bool timeout_braking_flag = false;
float odo_avg_speed, odo_theta, odo_x, odo_y;

#if PUBLISH_TO_RVIZ
ros::Publisher marker_pub;
#endif

enum Strategy {
  STRATEGY_NONE,
  STRATEGY_FOLLOW_BARRIER,
  STRATEGY_VORONOI,
  STRATEGY_PARKING,
  STRATEGY_GATE
};
Strategy strategy = STRATEGY_VORONOI;

void determineStrategy(ros::Publisher *mission_pub, ros::Publisher *brake_pub) {
  // Initialize output message
  auto_navi::driveMsg drivePoint;
  drivePoint.header.stamp = ros::Time::now();
  drivePoint.header.frame_id = "mission_control";

  // Determine what target to follow
  if (strategy == STRATEGY_VORONOI) {
    drivePoint.x = voronoi_dp.x;
    drivePoint.y = voronoi_dp.y;
    drivePoint.theta = voronoi_dp.theta;
    last_not_none_time = ros::Time::now();
  } else if (strategy == STRATEGY_FOLLOW_BARRIER) {
    drivePoint.x = follow_barrier_dp.x;
    drivePoint.y = follow_barrier_dp.y;
    drivePoint.theta = follow_barrier_dp.theta;
    last_not_none_time = ros::Time::now();
  } else if (strategy == STRATEGY_NONE) {
    drivePoint.x = odo_x + cos(odo_theta);
    drivePoint.y = odo_y + sin(odo_theta);
    drivePoint.theta = odo_theta;
  } else if (strategy == STRATEGY_PARKING) {
    drivePoint.x = parking_dp.x;
    drivePoint.y = parking_dp.y;
    drivePoint.theta = parking_dp.theta;
    last_not_none_time = ros::Time::now();
  } else if (strategy == STRATEGY_GATE) {
    drivePoint.x = gate_dp.x;
    drivePoint.y = gate_dp.y;
    drivePoint.theta = gate_dp.theta;
    last_not_none_time = ros::Time::now();
  }

  // Braking if "none" for 10 seconds, stops if it recieves smth else.
  ros::Duration ten_seconds(10.0);
  ros::Duration none_for = ros::Time::now() - last_not_none_time;
  if (none_for > ten_seconds) {
    dynamo_msgs::BrakeStepper brake;
    brake.brake_stepper_engaged = true;
    brake.brake_power = 10.0;
    brake_pub->publish(brake);
    ROS_INFO("Braking due to timeout");
    timeout_braking_flag = true;
  } else if (timeout_braking_flag && strategy != STRATEGY_NONE) {
    dynamo_msgs::BrakeStepper brake;
    brake.brake_stepper_engaged = false;
    brake.brake_power = 0.0;
    brake_pub->publish(brake);
    timeout_braking_flag = false;
    ROS_INFO("Stopped braking due to timeout");
  }

  // Output
  mission_pub->publish(drivePoint);
}

/*! Creates the vizualisation frame.
 * This function creates the visualization frame based on the world frame.
 * There is no translation between them, 2019-07-03, Asger.
 */
void static_tf2_vf_broadcaster() {
  static tf2_ros::StaticTransformBroadcaster static_vf_bc;
  geometry_msgs::TransformStamped static_tf;

  static_tf.header.stamp = ros::Time::now();
  static_tf.header.frame_id = "world";
  static_tf.child_frame_id = "visualization_frame";
  static_tf.transform.translation.x = 0.0;
  static_tf.transform.translation.y = 0.0;
  static_tf.transform.translation.z = 0.0;

  tf2::Quaternion tempquat;
  // Set the orientation of the world
  tempquat.setRPY(0, 0, 0);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(tempquat);
  static_tf.transform.rotation = quat_msg;
  static_vf_bc.sendTransform(static_tf);
}

// For visualizing the drivepoints in rviz
// Colors the different dp's different colors
void rvizzer(const auto_navi::driveMsg::ConstPtr &msg) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "visualization_frame"; //"base_link";
  marker.header.stamp = ros::Time::now();

  // Set the shape
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  static int id = 0;
  // id++; // Used to keep a lot of different dp's, change it in the if's

  marker.color.a = 1.0; // different from 0, unless you want it to be invisible.
  if (msg->header.frame_id == "voronoi_node") {
    marker.ns = "Voronoi drivepoint"; // Namespace, to easily check/uncheck
                                      // visible objects
    // Set the pose of the marker.
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = -1.0;
    // The orientation has to be given as a quaternion (instead of euler)
    tf2::Quaternion tempquat;
    tempquat.setRPY(0, 0, msg->theta); // Convert from euler to quaternions
    geometry_msgs::Quaternion quat_msg =
        tf2::toMsg(tempquat); // Convert from library type to message type
    marker.pose.orientation = quat_msg;
    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.id = 1; // id;

  } else if (msg->header.frame_id == "follow_barrier") {
    marker.ns = "FollowBarrier drivepoint"; // Namespace, to easily
                                            // check/uncheck visible objects
    // Set the pose of the marker.
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = -1.0;
    // The orientation has to be given as a quaternion (instead of euler)
    tf2::Quaternion tempquat;
    tempquat.setRPY(0, 0, msg->theta); // Convert from euler to quaternions
    geometry_msgs::Quaternion quat_msg =
        tf2::toMsg(tempquat); // Convert from library type to message type
    marker.pose.orientation = quat_msg;
    // Set the color
    if (strategy == STRATEGY_NONE) {
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
    } else {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
    }
    marker.id = 2; // id;

    // } else if (msg->header.frame_id == "parking ? ") { // Not on this topic,
    // so not implemented yet
  } else if (msg->header.frame_id == "gate_node") {
    marker.ns =
        "Gate drivepoint"; // Namespace, to easily check/uncheck visible objects
    // Set the pose of the marker.
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = -1.0;
    // The orientation has to be given as a quaternion (instead of euler)
    tf2::Quaternion tempquat;
    tempquat.setRPY(0, 0, msg->theta); // Convert from euler to quaternions
    geometry_msgs::Quaternion quat_msg =
        tf2::toMsg(tempquat); // Convert from library type to message type
    marker.pose.orientation = quat_msg;
    // Set the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.id = 3; // id;

  } else { // Others, not published
    return;
  }

  // Set the scale
  marker.scale.x = 1.0;
  marker.scale.y = 0.1;
  marker.scale.z = 0.2;
  // Set the lifetime
  marker.lifetime = ros::Duration(600, 0); //

  marker_pub.publish(marker);
}

void carPoseCallback(
    const std_msgs::Float32MultiArray::ConstPtr &msg_carPoseEstimate) {
  // carVal[0]: x
  // carVal[1]: y
  // carVal[3]: theta [rad] (orientation of car)
  // carVal[6]: drivendist

  // drivendist = msg_carPoseEstimate->data[6];   // updates global variable
  odo_x = msg_carPoseEstimate->data[0];
  odo_y = msg_carPoseEstimate->data[1];
  odo_theta = msg_carPoseEstimate->data[3];
  odo_avg_speed = msg_carPoseEstimate->data[4];
}

void drivePointCallback(const auto_navi::driveMsg::ConstPtr &msg) {
  if (msg->header.frame_id == "voronoi_node") {
    voronoi_dp.x = msg->x;
    voronoi_dp.y = msg->y;
    voronoi_dp.theta = msg->theta;

  } else if (msg->header.frame_id == "follow_barrier") {
    follow_barrier_dp.x = msg->x;
    follow_barrier_dp.y = msg->y;
    follow_barrier_dp.theta = msg->theta;

    // } else if (msg->header.frame_id == "parking ? ") {
  } else if (msg->header.frame_id == "gate_node") {
    gate_dp.x = msg->x;
    gate_dp.y = msg->y;
    gate_dp.theta = msg->theta;
    last_gate_time = msg->header.stamp; // From when the gate detector saw it

  } // Not covered : "mission_control" -> Is discarded

#if PUBLISH_TO_RVIZ
  rvizzer(msg);
#endif
}

void barriersVisibleCallback(const std_msgs::String::ConstPtr &msg) {
  // Possibilities: {None, Left, Right, Both}

  if (strategy == STRATEGY_PARKING) { // Don't exit parking mode
    return;
  }

  if (msg->data == "Left") { // Follow one barrier
    strategy = STRATEGY_FOLLOW_BARRIER;
  } else if (msg->data ==
             "Right") { // Same, because we follow the same drivepoint
    strategy = STRATEGY_FOLLOW_BARRIER;
  } else if (msg->data == "Both") { // Follow ye olde voronoi
    strategy = STRATEGY_VORONOI;
  } else { // Default, just drive straight
    strategy = STRATEGY_NONE;
  }

  // If we recieved a gate in the last 1 sec, we drive for that instead
  ros::Duration one_second(1.0);
  ros::Duration last_gate_msg_arrival = ros::Time::now() - last_gate_time;
  if (last_gate_msg_arrival < one_second) {
    strategy = STRATEGY_GATE;
  }
}

void parkingSpotCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) {
  strategy = STRATEGY_PARKING; // Set it once, keeps it forever.
  parking_dp.x = msg->polygon.points[0].x;
  parking_dp.y = msg->polygon.points[0].y;
  parking_dp.theta =
      follow_barrier_dp.theta; // TODO: fuck, how do we guess the orientation?
}

int main(int argc, char **argv) {
  // Important initializers
  ros::init(argc, argv, "mission_control");
  ros::NodeHandle n;

  // Pubs
  ros::Publisher mission_control_pub =
      n.advertise<auto_navi::driveMsg>("drive_points", 1);
  ros::Publisher brake_pub =
      n.advertise<dynamo_msgs::BrakeStepper>("cmd_brake_power", 1);
#if PUBLISH_TO_RVIZ
  marker_pub = n.advertise<visualization_msgs::Marker>("arrow_marker", 3);
#endif

  // Subs
  ros::Subscriber sub_drive_points =
      n.subscribe("drive_points", 100, drivePointCallback);
  ros::Subscriber sub_barriers_visible =
      n.subscribe("barriers_visible", 1, barriersVisibleCallback);
  ros::Subscriber sub_parking_spot =
      n.subscribe("/parking_spot_detection/corners", 1, parkingSpotCallback);
  ros::Subscriber sub_odo =
      n.subscribe("car_pose_estimate", 1000, carPoseCallback);

  // For the main loop
  int count = 0;       // Used to limit output messages
  float loopRate = 20; // loop rate in [Hz]
  ros::Rate r(loopRate);
  last_not_none_time = ros::Time::now();
  ROS_INFO("Starting mission control main loop");
  while (ros::ok()) {
    ros::spinOnce();

    determineStrategy(&mission_control_pub, &brake_pub);
#if PUBLISH_TO_RVIZ
    static_tf2_vf_broadcaster();
#endif
    r.sleep();
    ++count;
  }

  return 0;
}
