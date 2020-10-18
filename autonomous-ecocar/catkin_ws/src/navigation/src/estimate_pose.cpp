#include "ros/ros.h"
#include "navigation/global_variables.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>
#include <dynamo_msgs/TeensyRead.h>
#include <std_srvs/SetBool.h>
#include <fstream>
//added by Haotian Gao 08/10/2020
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
 #include <tf/transform_broadcaster.h>//using tf2 we can see in the tf-tree but we cannot use it so try the tf
#include <geometry_msgs/TransformStamped.h>
void tf2_broadcaster(std_msgs::Float32MultiArray odom_info);
void tf_broadcaster(std_msgs::Float32MultiArray odom_info);
//end

#define MAX_SPEED 7.0 // m/s
#define MAX_ACCELERATION 2.5 // m/s^2

#define LOG_TO_FILE false

#if LOG_TO_FILE
ofstream logFile(filename, ios::out | ios::binary); 
#endif

using namespace std;

double teensy_reading_time;
double previous_teensy_reading_time;
double time_since_last_teensy_update = 1;
double wheel_dist = 0;
double timestep=0;
double gyro_reading_time;
double previous_gyro_reading_time;
double time_since_last_gyro_update = 1;
double gyro_angle = 0;

// Reciever flags 
bool received_teensy = false;
bool received_gyro = false;

#define LOGFILE_NAME "/odoLog_%d.txt"
#define ANGLES_2_RADIANS 0.0174533

enum NodeState {
    NODE_STATE_FATAL_ERROR = -1, // For unrecoverable errors
    NODE_STATE_OK,               // When OK
    NODE_STATE_WAIT              // Node is not ready or trying to recover from an error
};

NodeState state;

class Measurements{
	public:
		double current_wheel_dist;
		double current_wheel_dist_timestamp;
		double previous_wheel_dist;
		double previous_wheel_dist_timestamp;

		
		double current_gyro_angle;
		double current_gyro_angle_timestamp;
		double previous_gyro_angle;
		double previous_gyro_angle_timestamp;

		Measurements(){
			current_wheel_dist = previous_wheel_dist = 0;
			current_wheel_dist_timestamp = previous_wheel_dist_timestamp = 0;

			current_gyro_angle = previous_gyro_angle = 0;
			current_gyro_angle_timestamp = previous_gyro_angle_timestamp = 0;
		};
		void initMeasurements(){
			current_wheel_dist = previous_wheel_dist = wheel_dist;
			current_wheel_dist_timestamp = previous_wheel_dist_timestamp = teensy_reading_time;
			current_gyro_angle = previous_gyro_angle = gyro_angle;
			current_gyro_angle_timestamp = previous_gyro_angle_timestamp = gyro_reading_time;
		};
		void updateMeasurements(){
			previous_wheel_dist = current_wheel_dist;
			previous_wheel_dist_timestamp = current_wheel_dist_timestamp;
			current_wheel_dist = wheel_dist;
			current_wheel_dist_timestamp = teensy_reading_time;

			previous_gyro_angle = current_gyro_angle;
			previous_gyro_angle_timestamp = current_gyro_angle_timestamp;
			current_gyro_angle = gyro_angle;
			current_gyro_angle_timestamp = gyro_reading_time;
		};
};

class MovingAverage{
	private:
		size_t size;
		std::vector<double> history;
		size_t top_index;
	public:
		double average;
		MovingAverage(size_t in_size = 3, double initial_values = 0.0){
			size = in_size;
			history.resize(size, initial_values);
			top_index = 0;
			average = initial_values;
		}
		void push(double new_value){
			top_index++;
			if(top_index >= size) top_index = 0;
			history.at(top_index) = new_value;

			average = 0;
			for (size_t i = 0; i < size; i++){
				average += history.at(i);
			}
			average = average/size;
			
			// average = std::accumulate( history.begin(), history.end(), 0.0)/size; 
		}
		double getPrevious(size_t previous_index){
			// previous_index = 0 is newest value
			if(previous_index >= size){
				ROS_WARN("Requested index is larger than moving average bounds!");
				return 0;
			} 
			if(previous_index > top_index){
				return history.at(top_index - previous_index + size);
			}
			return history.at(top_index - previous_index);
		}
};

class Odometry{
	public:
		double x, y, z;
		double orientation;
		double speed;
		double previous_speed;
		MovingAverage avg_speed;
		double driven_distance;
		double acceleration;
		double timestamp;
		Odometry(){
			x = pose_init[0];
			y = pose_init[1];
			z = pose_init[2];
			orientation = 0;
			speed = 0;
			previous_speed = 0;
			acceleration = 0;
			avg_speed = MovingAverage(3);
			driven_distance = 0;
			timestamp = ros::Time::now().toSec();
		}
		void updateOdometry(Measurements * m){
			double odo_distance = (m->current_wheel_dist - m->previous_wheel_dist);
			double odo_angle = (m->current_gyro_angle - m->previous_gyro_angle);
			
			// Wrap
			if(odo_angle > PI) odo_angle -= 2.0*PI;
			else if(odo_angle < -PI) odo_angle += 2.0*PI;
			
			// Update speed and acceleration
			if(odo_distance ==0){
			

			speed = previous_speed;						
			timestep += m->current_wheel_dist_timestamp - m->previous_wheel_dist_timestamp;
			}
			if(odo_distance !=0 && timestep >0 || timestep >0.5){
				previous_speed = speed;
				speed = odo_distance/(timestep);
				ROS_INFO("ododist: %f    timestep: %f     speed: %f",odo_distance,timestep,speed);	// debug
				if(fabs(speed) >= MAX_SPEED){
					ROS_WARN("Maximum speed exceded (%f m/s)", speed);
					ROS_WARN("Odo_distance = %f , timestep = %f", odo_distance,timestep);
					// Repair error
					speed = avg_speed.average;
					odo_distance = speed*timestep;
				}else{
					avg_speed.push(speed);
				}
				
				acceleration = (speed - previous_speed)/(timestep);
				//if(fabs(acceleration) >= MAX_ACCELERATION) ROS_WARN("Maximum acceleration exceded (%f m/s^2)", acceleration);
			}
			if(odo_distance !=0){
			timestep =0;			
}

			// If odometry is mounted on the "outer" wheel:
			if ((odo_mounting == 1 && m->current_gyro_angle >= m->previous_gyro_angle) || (odo_mounting == 0 && m->current_gyro_angle < m->previous_gyro_angle))
			odo_distance +=	0.5*dist_between_wheels * odo_angle;

			// If odometry is mounted on the "inner" wheel:
			if ((odo_mounting == 1 && m->current_gyro_angle <  m->previous_gyro_angle) || (odo_mounting == 0 && m->current_gyro_angle >=  m->previous_gyro_angle))
			odo_distance -=	0.5*dist_between_wheels * odo_angle;

			driven_distance += odo_distance;

			x += odo_distance * cos (orientation + odo_angle/2.0);
			y += odo_distance * sin (orientation + odo_angle/2.0);
			orientation += odo_angle;				
			// Wrapping angle around <-PI, +PI>:
			if (orientation > PI)	orientation = -2 * PI + orientation;
			else if (orientation < -PI) orientation = 2 * PI - orientation;

		}
		std_msgs::Float32MultiArray toMsg(){
			std_msgs::Float32MultiArray msg_odometry;
			msg_odometry.data.push_back(x);
			msg_odometry.data.push_back(y);
			msg_odometry.data.push_back(z);
			msg_odometry.data.push_back(orientation);
			msg_odometry.data.push_back(avg_speed.average);
			msg_odometry.data.push_back(acceleration);
			msg_odometry.data.push_back(driven_distance);			
			return msg_odometry;
		};
};

void teensyPosCallback (const dynamo_msgs::TeensyRead::ConstPtr& msg_teensy) {
	if(msg_teensy->distance_wheel < 0){
		ROS_WARN("Received bad teensy data: %f", msg_teensy->distance_wheel);
	}else{
		// Update timesstamps
		teensy_reading_time = msg_teensy->header.stamp.toSec(); //ros::Time::now().toSec();
		time_since_last_teensy_update = 0;

		// Update sensor measurements
		wheel_dist = msg_teensy->distance_wheel;
		received_teensy = true;
	}
}

void gyroAngleCallback (const std_msgs::Float32::ConstPtr& msg_gyro_angle) {
	// Update timesstamps
	gyro_reading_time = ros::Time::now().toSec();
	time_since_last_gyro_update = 0;

	// Update sensor measurements
	gyro_angle = - (msg_gyro_angle->data) * ANGLES_2_RADIANS; /*gyro is against the standard convention! Invert sign */
	received_gyro = true;
}

bool aliveCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if(!req.data) {
        ROS_INFO("Shutdown triggered by service!");
		#if LOG_TO_FILE
		logFile.close();
		#endif
		std::exit(EXIT_SUCCESS);
        state = NODE_STATE_FATAL_ERROR;
    }
    res.success = (state == NODE_STATE_OK);
    return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "pose_estimation");

	ros::NodeHandle n;

	ros::Publisher pub_carPoseEstimator = n.advertise<std_msgs::Float32MultiArray>("car_pose_estimate", 1);
	ros::Subscriber sub_steering_vel = n.subscribe("teensy_read", 1, teensyPosCallback);
	ros::Subscriber sub_steering_angle = n.subscribe("gyro_angle", 1, gyroAngleCallback);

    ros::ServiceServer service = n.advertiseService("estimate_pose_alive", aliveCallback);

	state = NODE_STATE_OK;

	double start_time = ros::Time::now().toSec();

	const float loop_rate = 50.0; // loop rate in [Hz]
	ros::Rate r(loop_rate);

	// Flags for warning
	bool teensy_update_stopped = false;
	bool gyro_update_stopped = false;

	// Init classes
	Measurements measurements;
	Odometry odometry;
	
	// Wait for first sensor measurements
	if(!received_teensy || !received_gyro) ROS_INFO("Waiting for data...");
	while (ros::ok()){
		if(received_teensy && received_gyro) break;
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("Got initial data!");

	// Logging to file
	char filename[100];
	sprintf(filename, LOGFILE_NAME, (int)ros::WallTime::now().toSec() - 1494796385);
	//ofstream logFile(filename, ios::out | ios::binary); // TODO: Check if can avoided by if(LOG_FILE) 
	#if LOG_TO_FILE
	logFile << "Time,Estimate x,Estimate y,Estimate theta\n";
	#endif

	// Initialize measurements & odometry
	measurements.initMeasurements();

	while (ros::ok()) {
		if(state == NODE_STATE_FATAL_ERROR) break;

		if(received_teensy && received_gyro){
			// Inform if stream started again
			if(teensy_update_stopped){
				ROS_INFO("Teensy streaming started again!");
				teensy_update_stopped = false;
			}
			if(gyro_update_stopped){
				ROS_INFO("Gyro streaming started again!");
				gyro_update_stopped = false;
			}
			// Retrieve sensor measurements
			measurements.updateMeasurements();

			// Update odometry
			odometry.updateOdometry(&measurements);

			// Publish odometry message
			pub_carPoseEstimator.publish(odometry.toMsg()); // TODO: ADD timestamp!
            //added by Haotian Gao 08/10/2020
            tf2_broadcaster(odometry.toMsg());
			//tf_broadcaster(odometry.toMsg());
            //end
			#if LOG_TO_FILE
			logFile << ros::Time::now() << "," <<  odometry.x << "," << odometry.y << "," << odometry.orientation << "\n";
			#endif

			// ROS_INFO("Position estimate: [X: %f\tY: %f\tOrientation: %f\tSpeed: %f\tDistance_raw: %f\tDistance: %f]", 
			// odometry.x, odometry.y, odometry.orientation, 
			// odometry.speed, measurements.current_wheel_dist, odometry.driven_distance);

			// Reset flags
			received_teensy = false;
			received_gyro = false;
		}else{
			// Prompt if stopped
			time_since_last_teensy_update += 1.0 / loop_rate;
			if (time_since_last_teensy_update >= 1.0 && !teensy_update_stopped){
				ROS_WARN("Warning: Teensy streaming stopped!");
				teensy_update_stopped = true;
			}
			time_since_last_gyro_update += 1.0 / loop_rate;
			if (time_since_last_gyro_update >= 1 && !gyro_update_stopped){
				ROS_WARN("Warning: Gyro angle streaming stopped!");
				gyro_update_stopped = true;
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
	#if LOG_TO_FILE
	logFile.close();
	#endif

	return 0;
}
//added by Haotian Gao 08/10/2020
void tf2_broadcaster(std_msgs::Float32MultiArray odom_info){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = odom_info.data[0];
    transformStamped.transform.translation.y = odom_info.data[1];
    transformStamped.transform.translation.z = odom_info.data[2];
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_info.data[3]);//orientation
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
}
void tf_broadcaster(std_msgs::Float32MultiArray odom_info){
    static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(odom_info.data[0], odom_info.data[1], odom_info.data[2]) );
            tf::Quaternion q;
            q.setRPY(0, 0, odom_info.data[3]);//orientation
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom"));
}

//end