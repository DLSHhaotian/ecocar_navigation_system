#include <stdio.h>
//#include <tchar.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//#include "opencv2\core\core.hpp"
//#include "opencv2\calib3d\calib3d.hpp"

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <numeric> 
#include <algorithm>

#include "parking_spot_image_processing_block.h"

#define WINDOW_SIZE 300
using namespace std;
using namespace cv;

static Mat img_global;
static Mat M_global;

class ImgParam {

public:


  int pers_x = 600;
  int pers_y = 1000;

  int gauss_sigma = 5;
  int dilatile1 = 2;
  int erode1 = 2;

  double approxPoly_epsilon = 0.03;
  int min_area = 4000;
  int max_area = 40000;

  double maxCosine = 0.4;
	
  int parking_spot_difference = 80;
  int parking_spot_color_difference = 70;
  double max_car_angle = 0.5;

  int blur = 5;
  // Kernel size must be odd and positive
  int gauss_kernel_size = 35;

  int hmin = 95;
  // 179 is max for hue value
  int hmax = 115;
  int smin = 0;
  int smax = 255;
  int vmin = 0;
  int vmax = 210;



  void update() {
	  hmin = getTrackbarPos("Low", "Hue");
	  hmax = getTrackbarPos("High", "Hue");
	  smin = getTrackbarPos("Low", "Saturation");
	  smax = getTrackbarPos("High", "Saturation");
	  vmin = getTrackbarPos("Low", "Value");
	  vmax = getTrackbarPos("High", "Value");
  };


}static param;

static double angle(Point pt1, Point pt2, Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}




// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
static void findSquares(const Mat& image, vector<vector<Point> >& squares)
{
  squares.clear();

  Mat pyr, timg;

  vector<vector<Point> > contours;


  findContours(image, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

  vector<Point> approx;

  // test each contour
  for (size_t i = 0; i < contours.size(); i++)
  {
    // approximate contour with accuracy proportional
    // to the contour perimeter
    approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*param.approxPoly_epsilon, true);
    //approxPolyDP(approx, approx, arcLength(Mat(contours[i]), true)*0.02, true);

    //drawContour(approx);


    // square contours should have 4 vertices after approximation
    // relatively large area (to filter out noisy contours)
    // and be convex.
    int contour_area = fabs(contourArea(Mat(approx)));
    if (approx.size() == 4 &&
      contour_area > param.min_area &&
      contour_area < param.max_area &&
      isContourConvex(Mat(approx)))
    {
      double maxCosine = 0;

      for (int j = 2; j < 5; j++)
      {
        // find the maximum cosine of the angle between joint edges
        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
        maxCosine = MAX(maxCosine, cosine);
      }

      // if cosines of all angles are small
      // (all angles are ~90 degree) then write quandrange
      // vertices to resultant sequence
      if (maxCosine < param.maxCosine)
        squares.push_back(approx);
    }
  }
}


static int distance(Point p1, Point p2) {

  return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


static void sort_verticles(vector<vector<Point> > &squares) {

  vector<vector<Point> > squares_sorted;
  for (int i = 0; i < squares.size(); i++) {
    vector<Point> current_square;

    // Iterate four times
    for (int j = 0; j < 4; j++) {
      int max_dist = 0;
      int max_y = 0;
      int min_x = 100000;
      int index;

      // Iterate all points in square
      for (int k = 0; k < squares[i].size(); k++) {
        switch (j) {
          case 0:
            if ((squares[i][k].x + squares[i][k].y) > max_dist) {
              index = k;
              max_dist = (squares[i][k].x + squares[i][k].y);
            }
            break;
          case 1:
            if (squares[i][k].y > max_y) {
              index = k;
              max_y = squares[i][k].y;
            }
            break;
          case 2:
            if (squares[i][k].x < min_x) {
              index = k;
              min_x = squares[i][k].x;
            }
            break;
          case 3:
            index = 0;
            break;
        }
      }
      current_square.push_back(squares[i][index]);
      squares[i].erase(squares[i].begin() + index);
    }
    squares_sorted.push_back(current_square);
  }

  squares = squares_sorted;
}

static int square_difference(vector<Point> s1, vector<Point> s2) {

  int sum = 0;
  if (s1.size() == s2.size()) {
    for (int i = 0; i < s1.size(); i++)
      sum += distance(s1[i], s2[i]);
  }
  return sum;
}


static vector<Point> average_duplicate_spots(vector<Point> s1, vector<Point> s2) {

  vector<Point> square_average;
  for (int i = 0; i < s1.size(); i++) {
    square_average.push_back(Point((s1[i].x+s2[i].x)/2, (s1[i].y + s2[i].y) / 2));
  }

  return square_average;
}

static void eliminate_duplicates(vector<vector<Point>> &squares) {


  if (squares.size() > 1) {

    for (int i = 0; i < squares.size()-1; i++) {
      for (int j = i + 1; j < squares.size(); j++) {
        if (param.parking_spot_difference > square_difference(squares[i], squares[j])) {
          squares.push_back(average_duplicate_spots(squares[i], squares[j]));
          squares.erase(squares.begin() + j);
          squares.erase(squares.begin() + i);
        }
      }
    }
  }
}



static void drawSquares(Mat& image, const vector<vector<Point> >& squares)
{
  for (size_t i = 0; i < squares.size(); i++)
  {
    const Point* p = &squares[i][0];
    int n = (int)squares[i].size();
    polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
  }

}

static void lines_ratio_check(vector<vector<Point> > &squares){

	for (int i = 0; i < squares.size(); i++) {

		if (((double)distance(squares[0][0], squares[0][1])/(double)distance(squares[0][1], squares[0][2])) < 5.0 ||
				((double)distance(squares[0][0], squares[0][1])/(double)distance(squares[0][1], squares[0][2])) > 7.0)
					squares.erase(squares.begin() + i);
	}
}

/*
static void callback(int, void*)
{
	Mat call_img = img_global.clone();
	Mat call_img_gauss, call_img_canny, call_img_HSV, temp, call_img_HSV_out;
	param.update();

	Mat image_filtered;
  GaussianBlur(call_img, image_filtered, Size(5,5), 0.2*param.gauss_sigma);

	vector<Mat> hsvChannels(3);
	cvtColor(call_img, call_img_HSV, cv::COLOR_BGR2HSV);
	split(call_img_HSV, hsvChannels);
	inRange(hsvChannels[0], param.hmin, param.hmax, hsvChannels[0]);
	imshow("Hue", hsvChannels[0]);
	inRange(hsvChannels[1], param.smin, param.smax, hsvChannels[1]);
	imshow("Saturation", hsvChannels[1]);
	inRange(hsvChannels[2], param.vmin, param.vmax, hsvChannels[2]);
	imshow("Value", hsvChannels[2]);

	bitwise_and(hsvChannels[0], hsvChannels[1], temp);
	bitwise_and(temp, hsvChannels[2], call_img_HSV_out);
	imshow("HSV", call_img_HSV_out);
	waitKey(0);

}*/

vector<vector<Point2f> > parking_spot_detection_block(Mat &image, double car_angle, Mat &H, int img_out_width, int img_out_height)
{

	if(car_angle > param.max_car_angle || car_angle < -param.max_car_angle){
		vector<vector<Point2f> > nothing;
		return nothing;
	}
	
  	Mat image_filtered;
  	GaussianBlur(image, image_filtered, Size(5,5), 0.4*param.gauss_sigma);
  	//img_global = image.clone();

  	vector<Mat> hsvChannels(3);
  	Mat img_HSV;
  	cvtColor(image_filtered, img_HSV, cv::COLOR_BGR2HSV);
  	split(img_HSV, hsvChannels);


  	inRange(hsvChannels[0], param.hmin, param.hmax, hsvChannels[0]);
  	/*namedWindow("Hue", WINDOW_NORMAL);
  	resizeWindow("Hue", WINDOW_SIZE, WINDOW_SIZE);
  	moveWindow("Hue", 0, WINDOW_SIZE);
  	imshow("Hue", hsvChannels[0]);*/
  	//createTrackbar("Low", "Hue", &(param.hmin), 180, callback);
  	//createTrackbar("High", "Hue", &(param.hmax), 180, callback);


	inRange(hsvChannels[2], param.vmin, param.vmax, hsvChannels[2]);
	/*namedWindow("Value", WINDOW_NORMAL);
	resizeWindow("Value", WINDOW_SIZE, WINDOW_SIZE);
	moveWindow("Value", 2 * WINDOW_SIZE, WINDOW_SIZE);
	imshow("Value", hsvChannels[2]);*/
	//createTrackbar("Low", "Value", &(param.vmin), 255, callback);
	//createTrackbar("High", "Value", &(param.vmax), 255, callback);

	Mat temp_mask, img_HSV_out;
	  bitwise_and(hsvChannels[0], hsvChannels[2], img_HSV_out);
	/*namedWindow("HSV", WINDOW_NORMAL);
	resizeWindow("HSV", WINDOW_SIZE, WINDOW_SIZE);
	moveWindow("HSV", 0, 2 * WINDOW_SIZE);
	imshow("HSV", img_HSV_out);*/


  	Mat img_canny_morph;
  	dilate(img_HSV_out, img_canny_morph, Mat(), Point(-1, -1), param.dilatile1, 2);

  	erode(img_canny_morph, img_canny_morph, Mat(), Point(-1, -1), param.erode1, 2);

	vector<vector<Point> > squares;
  	findSquares(img_canny_morph, squares);
  	sort_verticles(squares);
  	eliminate_duplicates(squares);
	lines_ratio_check(squares);

	for(int i=0; i<squares.size(); i++){


		if(squares[i][3].y < 0.35*image.rows)
			squares.erase(squares.begin() + i);

	}

  /*drawSquares(image, squares);
	namedWindow("Output",WINDOW_NORMAL);
	imshow("Output", image);
  waitKey(10);*/

	// Change Point to Point2f
	vector<vector<Point2f> > squares_out;
	for(int i=0; i < squares.size(); i++){
		vector<Point2f> temp_square;
		for(int j=0; j < squares[i].size(); j++){
			Point2f p = squares[i][j];
			temp_square.push_back(p);
		}
		squares_out.push_back(temp_square);
	}


	for (int i = 0; i < squares.size(); i++) {
		perspectiveTransform(squares_out[i], squares_out[i], H);
	}


	/*cout << "squares_out.size() =  " << to_string(squares_out.size()) << endl;
	for (int i = 0; i < squares_out.size(); i++) {
		cout << "Parking spot: " << endl;
		cout << "squares_out[i].size() =  " << to_string(squares_out[i].size()) << endl;
		for (int j=0; j < squares_out[i].size(); j++){
			cout << to_string(squares_out[i][j].x) << " " << to_string(squares_out[i][j].y) << endl;	
		}
	}*/



  return squares_out;
}

