#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PolygonStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include <algorithm>
#include <dynamo_msgs/Speak.h>

#include "parking_spot_image_processing.h"
#include "parking_spot_image_processing_block.h"

using namespace std;
using namespace cv;


// CONSTANTS

bool debug = 0;

// files
string package_path = ros::package::getPath("parking_spot_detection");
string calibration_img_path = package_path + "/../../img/top_view_calibration.jpg";

// topics
const string input_img_topic = "/usb_cam/image_rect_color"; //"camera"; //
const string spot_corners_topic = "/parking_spot_detection/corners";

// top view image scale and position
const int ch_spacing_px = 4; // pixels per 1 squere of checkerboard
const int img_out_height = 1000;
const int img_out_width = 600;
const Point2f ch_lower_left_px = Point2f(img_out_width / 2, img_out_height - 5 * ch_spacing_px);

const double ch_spacing_m = 0.108; // [m]
const Point2f ch_lower_left_m = Point2f(5, 0);

const double scale_m_px = ch_spacing_m / ch_spacing_px;

// checkerboard size
const int ch_height = 6;
const int ch_width = 8;

// GLOBAL VARIABLES
Mat H; // for storing homography
Mat top_view_to_car; // top view image to car coordinates transformation

double pose[3] = {0, 0, 0};
double pose_last_img[3] = {0, 0, 0};

vector<Point2f> spot_global_avg;
int no_of_detections = 0;

ros::Publisher pub_rect;
ros::Publisher pub_speak;

bool img_received = false;

// camera input
int img_in_height = 0;
int img_in_width = 0;

int top_view_calibration();
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void rosbagCallback (const std_msgs::Float32MultiArray::ConstPtr& msg_pose_est) {
  pose[0] = msg_pose_est->data[0]; //x
  pose[1] = msg_pose_est->data[1]; //y
  pose[2] = msg_pose_est->data[3]; //theta

}

class ParkingSpots {

public:

  struct Spot {
    Point2f center;
    Point2f front;
  };

  vector<vector<Point2f> > raw_detected_corners;
  vector<vector<Point2f> > raw_detected_corners_global;
  vector<Spot> raw_detected_global;


  double categorizing_dist_similarity = 2.5;
  vector<vector<Spot> > history_spots;

  int validation_size = 3;
  vector<Spot> validated_spots;
  vector<int> validated_spots_importance;

  bool multiple_spots_challange = false;
  // Desired spot to park in, range: 0 to 3
  // Remember off by one mistake
  int spot_number = 2;

  double dist_block_detection;

  vector<Spot> spot_to_send;
  bool spot_found_by_lines = 1;
  bool spot_sent_by_lines = 0;

  void local_to_global() {

    // transform points from image coordinates to global coordinates
    for (vector<vector<Point2f> >::iterator it = raw_detected_corners.begin(); it != raw_detected_corners.end(); ++it)
    {
      vector<Point2f> spot_car, spot_global;

      // transform from image to local
      perspectiveTransform(*it, spot_car, top_view_to_car);

      // transform from local to global
      for (vector<Point2f>::iterator it2 = spot_car.begin(); it2 != spot_car.end(); ++it2)
      {
        //cout << "Parking spot x: " << to_string((*it2).x) << " parking spot y: " << to_string((*it2).y) << endl;
        Point2f p;
        // from local to global, whereever:
        p.x = cos(pose_last_img[2]) * (*it2).x - sin(pose_last_img[2]) * (*it2).y + pose_last_img[0];
        p.y = sin(pose_last_img[2]) * (*it2).x + cos(pose_last_img[2]) * (*it2).y + pose_last_img[1];
        spot_global.push_back(p);
      }


      raw_detected_corners_global.push_back(spot_global);
    }
  }

  void corners_to_centers() {


    for (vector<vector<Point2f> >::iterator it = raw_detected_corners_global.begin(); it != raw_detected_corners_global.end(); ++it) {

      float avg_x = 0;
      float avg_y = 0;
      for (vector<Point2f>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2)
      {
        avg_x += (*it2).x;
        avg_y += (*it2).y;
      }

      avg_x /= (*it).size();
      avg_y /= (*it).size();

      Spot new_spot;
      if (spot_found_by_lines == 1) {
        new_spot.center = Point2f(avg_x, avg_y);
        new_spot.front = Point2f( ((*it)[0].x + (*it)[1].x) / 2, ((*it)[0].y + (*it)[1].y) / 2 );
      }
      else {
        double y_multiplier = 0.8;
        new_spot.front = Point2f( ((*it)[0].x + (*it)[1].x) / 2 - 6, ((*it)[0].y + (*it)[1].y) / 2 * y_multiplier );
        new_spot.center = Point2f( ((*it)[0].x + (*it)[1].x) / 2 - 4, ((*it)[0].y + (*it)[1].y) / 2 * y_multiplier);
      }
      raw_detected_global.push_back(new_spot);
    }
  }

  double distance(Point2f p1, Point2f p2) {

    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
  }

  void add_new_spots() {

    for (int i = 0; i < raw_detected_global.size(); i++) {

      if (history_spots.empty()) {
        //        cout << "history_spots_centers.empty() in " + to_string(i) << endl;
        vector<Spot> temp;
        Spot temp_spot = raw_detected_global[i];
        temp.push_back(temp_spot);
        history_spots.push_back(temp);

      }
      else {
        for (int j = 0; j < history_spots.size(); j++)
        {
          if (distance(raw_detected_global[i].center, history_spots[j][0].center) < categorizing_dist_similarity) {

            Spot temp_spot = raw_detected_global[i];
            history_spots[j].push_back(temp_spot);
            break;
          }
          else {
            // If no similar parking spot exists, then push current spot as a new group of spots
            if (j == history_spots.size() - 1) {

              vector<Spot> temp;
              Spot temp_spot = raw_detected_global[i];
              temp.push_back(temp_spot);
              history_spots.push_back(temp);
              break;
            }
          }
        }
      }
    }

    /*for(int i = 0; i < history_spots.size(); i++){
        cout << "\nHistory spots group " + to_string(i) + ": " << endl;

        for(int j = 0; j < history_spots[i].size(); j++){
            cout << to_string(j) + " parking spot: center " + to_string(history_spots[i][j].center.x) + " " + to_string(history_spots[i][j].center.y) +
                    " front: " + to_string(history_spots[i][j].front.x) + " " + to_string(history_spots[i][j].front.y) << endl;
        }
        cout << endl;
      }*/
  }

  void erase_old_spots() {

    for (int i = 0; i < history_spots.size(); i++) {
      if (history_spots[i].size() > 100) {
        history_spots[i].erase(history_spots[i].begin());
      }
    }
  }

  void validate_spots() {

    for (int i = 0; i < history_spots.size(); i++) {
      if (history_spots[i].size() > validation_size) {

        double avg_x = 0;
        double avg_y = 0;
        Spot temp_spot;

        for (int j = 0; j < history_spots[i].size(); j++) {
          avg_x += history_spots[i][j].center.x;
          avg_y += history_spots[i][j].center.y;
        }
        avg_x /= history_spots[i].size();
        avg_y /= history_spots[i].size();
        temp_spot.center = Point2f(avg_x, avg_y);

        avg_x = 0;
        avg_y = 0;

        for (int j = history_spots[i].size() - 1; j > history_spots[i].size() - 4; j--) {
          avg_x += history_spots[i][j].front.x;
          avg_y += history_spots[i][j].front.y;
        }

        avg_x /= 3;
        avg_y /= 3;
        temp_spot.front = Point2f(avg_x, avg_y);

        if (avg_y < 8 && avg_y > -8) {
          validated_spots.push_back(temp_spot);
        }

        validated_spots_importance.push_back(int(history_spots[i].size()));
      }
    }


    /*for(int j = 0; j < validated_spots.size(); j++){
        cout << "Validated spot: " + to_string(j) + " parking spot: center " + to_string(validated_spots[j].center.x) + " " + to_string(validated_spots[j].center.y) +
                " front: " + to_string(validated_spots[j].front.x) + " " + to_string(validated_spots[j].front.y) << "\n\n";
    }*/


  }

  struct mysort {
    bool operator() (Spot p1, Spot p2) {
      return sqrt(pow((p1.center.x), 2) + pow(p1.center.y, 2)) < sqrt(pow(p2.center.x, 2) + pow(p2.center.y, 2));
    }
  } my_sort_object;

  void find_spot_multiple_challange() {

    if (!validated_spots.empty()) {

      vector<Spot> sorted_validated_spots = validated_spots;
      sort(sorted_validated_spots.begin(), sorted_validated_spots.end(), my_sort_object);

      if (sorted_validated_spots.size() > spot_number)
        spot_to_send.push_back(sorted_validated_spots[spot_number]);
    }
  }


  void send_spot() {

    // publish points
    if (spot_to_send.empty())
      return;

    if (spot_found_by_lines == 1)
      spot_sent_by_lines = 1;

    geometry_msgs::PolygonStamped spot;
    spot.header.frame_id = "visualization_frame";
    spot.header.stamp = ros::Time::now();

    geometry_msgs::Point32 p_center;
    p_center.x = spot_to_send[0].center.x;
    p_center.y = spot_to_send[0].center.y;
    p_center.z = 0;
    spot.polygon.points.push_back(p_center);

    geometry_msgs::Point32 p_front;
    p_front.x = spot_to_send[0].front.x;
    p_front.y = spot_to_send[0].front.y;
    p_front.z = 0;
    spot.polygon.points.push_back(p_front);

    pub_rect.publish(spot);
  }

  void send_best_spot()
  {
    int best = 0;
    for (int i = 0; i < validated_spots_importance.size(); i++)
    {
      if (validated_spots_importance[i] >= validated_spots_importance[best])
      {
        best = i;
      }
    }
    spot_to_send.push_back(validated_spots[best]);
  }

  void update_and_send() {

    local_to_global();
    corners_to_centers();
    add_new_spots();
    erase_old_spots();
    validate_spots();
    if (multiple_spots_challange) {
      find_spot_multiple_challange();
    }
    else if (validated_spots.size() > 0) {
      send_best_spot();
    }

    send_spot();

    /*if(spot_to_send.size() > 0){
       cout << "SPOT TO SEND: center " + to_string(spot_to_send[0].center.x) + " " + to_string(spot_to_send[0].center.y) +
                " front: " + to_string(spot_to_send[0].front.x) + " " + to_string(spot_to_send[0].front.y) << "\n\n";
      }*/
    //   else{
    //     cout << "NO SPOT TO SEND" << endl;
    //   }


    raw_detected_corners.clear();
    raw_detected_corners_global.clear();
    raw_detected_global.clear();
    validated_spots.clear();
    validated_spots_importance.clear();
    spot_to_send.clear();

  }

} parking_detector;




int main( int argc, char** argv )
{
  // ROS initialization
  ros::init(argc, argv, "parking_spot_detection");
  ros::NodeHandle nh("~");
  nh.param<double>("plan_b", parking_detector.dist_block_detection, 16.0);
  image_transport::ImageTransport it(nh);
  double rate = 30;


  pub_rect = nh.advertise<geometry_msgs::PolygonStamped>(spot_corners_topic, 1000);
  ros::Subscriber sub_pose_rosbag = nh.subscribe("/car_pose_estimate", 1, rosbagCallback);
  image_transport::Subscriber sub_img = it.subscribe(input_img_topic, 1, imageCallback);

  // camera connection check
  pub_speak = nh.advertise<dynamo_msgs::Speak>("/speak", 1);
  dynamo_msgs::Speak speak_msg_cam;
  speak_msg_cam.sound = 0;
  speak_msg_cam.textMessage = "Camera disconnected!";
  ros::Time start_time = ros::Time::now();

  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    ros::spinOnce();
    if ((ros::Time::now() - start_time).toSec() > 2 && !img_received)
    {
      pub_speak.publish(speak_msg_cam);
    }
    loop_rate.sleep();
  }

  return 0;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  pose_last_img[0] = pose[0];
  pose_last_img[1] = pose[1];
  pose_last_img[2] = pose[2];

  // convert to opencv Mat
  Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

  if (!img_received)
  {
    img_received = true;
    img_in_height = img.size().height;
    img_in_width = img.size().width;
    ROS_INFO_STREAM("Input image size " << img.size());
    if (top_view_calibration() != 0)
    {
      ROS_ERROR("Top view calibration error!");
      //ros::shutdown(); // Removed as a hotfix 2019
    }
  }



  ROS_INFO("Processing new frame...");
  vector<vector<Point2f> > detetcted_corners_float;

  if (pose[0] > parking_detector.dist_block_detection && parking_detector.spot_sent_by_lines == 0) {

    detetcted_corners_float = parking_spot_detection_block(img, pose[2], H, img_out_width, img_out_height);
    if (!detetcted_corners_float.empty()) {
      parking_detector.spot_found_by_lines = 0;
    }
  }

  else {

    Mat img_warped;
    warpPerspective(img, img_warped, H, Size(img_out_width, img_out_height));
    vector<vector<Point> > detetcted_corners_int = parking_spot_detection(img_warped, pose[2]);

    // transform points from image coordinates to global coordinates
    for (vector<vector<Point> >::iterator it = detetcted_corners_int.begin(); it != detetcted_corners_int.end(); ++it)
    {
      vector<Point2f> temp;
      // convert to Point2f
      for (vector<Point>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2)
      {
        temp.push_back(*it2);
      }
      detetcted_corners_float.push_back(temp);
    }
    if (debug)
    {
      imshow("in", img);
      imshow("warped", img_warped);
      waitKey(1);
    }
  }

  if (!detetcted_corners_float.empty()) {
    parking_detector.raw_detected_corners = detetcted_corners_float;
    parking_detector.update_and_send();
  }
}

int top_view_calibration()
{
  Mat img = imread(calibration_img_path);
  if (img.empty())
  {
    ROS_ERROR("Could not open the image!");
    return -1;
  }

  if (img.size().height != img_in_height)
  {
    ROS_INFO("Top-view calibration image scaled, calibration quality may suffer.");
    resize(img, img, Size(img_in_width, img_in_height), 0, 0, INTER_LANCZOS4);
  }


  // get corners from calibration image
  vector<Point2f> corners_in;
  bool found = findChessboardCorners(img, cvSize(ch_height, ch_width), corners_in);
  if (!found)
  {
    ROS_ERROR("Corners not found!");
    return -1;
  }

  // generate destination corners
  vector<Point2f> corners_out;
  for (int i = 0; i < ch_width; i++)
  {
    for (int j = 0; j < ch_height; j++)
    {
      Point2f p = Point2f(ch_lower_left_px.x + (ch_width - 1 - i) * ch_spacing_px, ch_lower_left_px.y + (j - ch_height - 1) * ch_spacing_px);
      corners_out.push_back(p);
    }
  }

  H = findHomography(corners_in, corners_out);

  // get real position of lower left corner in px
  vector<Point2f> corners_top;
  perspectiveTransform(corners_in, corners_top, H);

  // calculate top-view image coordinates to car coordinates transform
  Point2f ch_lower_left_px_real = corners_top[ch_width * ch_height - 1]; // position after transform is different then defined, probably because of approximations/corrections based on other matched points during homography estimation, I need to get correct position to calculate distance properly
  top_view_to_car = (Mat_<double>(3, 3) << 0, -scale_m_px, scale_m_px * ch_lower_left_px_real.y + ch_lower_left_m.x,   -scale_m_px, 0, scale_m_px * ch_lower_left_px_real.x,   0, 0, 1);

//  if (debug)
//  {
//    imshow("cal", img);
//    // draw detected points on image and show for inspection
//    for (std::vector<Point2f>::const_iterator i = corners_in.begin(); i != corners_in.end(); ++i)
//    {
//      circle(img, *i, 4, cvScalar(0,0,255), 2);
//    }
//    imshow("cal2", img);
//    waitKey(0);
//  }
  ROS_INFO("Top-view calibrated.");

  return 0;
}
