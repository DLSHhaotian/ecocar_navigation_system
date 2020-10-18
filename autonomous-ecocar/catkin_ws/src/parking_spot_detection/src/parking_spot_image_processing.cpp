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

#include "parking_spot_image_processing.h"

using namespace std;
using namespace cv;

static Mat img_global;
static Mat M_global;

class ImgParam {

public:


  int pers_x = 600;
  int pers_y = 1000;
  int segmentation_mask_x = 36;
  int segmentation_mask_y = 140;

  int grabCutIterations = 2;

  int lowCan = 850;
  int highCan = 900;
  int canny_kernel_size = 5;

  int bilateral_size = 3;
  int bilateral_sigma_color = 14;
  int bilateral_sigma_space = 120;
  int gauss_sigma = 5;
  int dilatile1 = 6;
  int dilatile2 = 7;
  int erode1 = 1;
  int erode2 = 3;

  double approxPoly_epsilon = 0.06;
  int min_area = 4000;
  int max_area = 14000;

  double maxCosine = 0.8;
	
  int parking_spot_difference = 80;
  int parking_spot_color_difference = 120;
  double max_car_angle = 0.5;
	double lines_ratio = 3.0;


}static param;

static Mat1b segmentation_mask(size_t img_cols, size_t img_rows, int down_sampling_factor, double angle) {

  cv::Mat1b markers((int)(img_rows/ down_sampling_factor), (int)(img_cols/ down_sampling_factor));
  // let's set all of them to possible not road terrain
  markers.setTo(cv::GC_PR_BGD);

  /*cv::Mat1b fg_pr_seed = markers(Rect((int)(img_cols / down_sampling_factor) / 2 - (int)(22 * 4 / down_sampling_factor),
                    (int)(img_rows / down_sampling_factor) - (int)(260 * 4 / down_sampling_factor),
                  2 * (int)(22 * 4 / down_sampling_factor),
                    (int)(260 * 4 / down_sampling_factor)));
    // mark it as likely road
  fg_pr_seed.setTo(cv::GC_PR_FGD);*/

  // region of interest
  cv::Mat1b fg_seed_1 = markers(Rect((int)(img_cols / down_sampling_factor)/2 - (int)(param.segmentation_mask_x*4 / down_sampling_factor),
                     (int)(img_rows / down_sampling_factor) - (int)(param.segmentation_mask_y * 4 / down_sampling_factor),
                   2*(int)(param.segmentation_mask_x * 4 / down_sampling_factor),
                     (int)(param.segmentation_mask_y * 4 / down_sampling_factor)));

  // mark it as road
  fg_seed_1.setTo(cv::GC_FGD);

  //// select first rows of the image as background
  //cv::Mat1b bg_seed = markers(cv::Range(0, (int)(24 / down_sampling_factor)), cv::Range::all());
  //bg_seed.setTo(cv::GC_BGD);

  double angle_degrees = angle*180/3.14;
  Mat R = getRotationMatrix2D(Point2f((int)(img_cols/ (2*down_sampling_factor)), (int)(img_rows/ down_sampling_factor)), -angle_degrees, 1);
  warpAffine(markers, markers, R, Size((int)(img_cols/ down_sampling_factor), (int)(img_rows/ down_sampling_factor)), INTER_LINEAR, BORDER_CONSTANT, 					cv::GC_PR_BGD);

  return markers;
}




static void img_segmentation(Mat img, Mat1b mask, Mat1b &mask_upscale_out, int iterations) {

  int down_sampling_factor = 4;
  Mat img_down = img;
  pyrDown(img, img_down, Size(img.cols / 2, img.rows / 2));
  pyrDown(img_down, img_down, Size(img_down.cols / 2, img_down.rows / 2));

  cv::Mat bgd, fgd;
  cv::grabCut(img_down, mask, cv::Rect(), bgd, fgd, iterations, cv::GC_INIT_WITH_MASK);

  // let's get all foreground and possible foreground pixels
  cv::Mat1b mask_out = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);

  pyrUp(mask_out, mask_upscale_out, Size(mask_out.cols * 2, mask_out.rows * 2));
  pyrUp(mask_upscale_out, mask_upscale_out, Size(mask_upscale_out.cols * 2, mask_upscale_out.rows * 2));
}


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

		int max_line_len=0;
		int min_line_len=100000;
		
		for(int j=0; j<squares[i].size()-1; j++){
			
			int dist = distance(squares[i][j], squares[i][j+1]);
			if(dist > max_line_len)
				max_line_len = dist;
			if(dist < min_line_len)
				min_line_len = dist;

		}


		if ((double)max_line_len / (double)min_line_len > param.lines_ratio)
			squares.erase(squares.begin() + i);
	}
}


static void color_check(Mat image_pers, vector<vector<Point>> &squares, Scalar road_avr_intensity) {

	
	for (int i = 0; i < squares.size(); i++) {

		cv::Mat square_mask(param.pers_y, param.pers_x, CV_8UC1, cv::Scalar(0));
		//square_mask.setTo(0);
		drawContours(square_mask, squares, i, Scalar(1), CV_FILLED, 8);

		Scalar parnking_spot_avr_intensity = mean(image_pers, square_mask);
		int color_dist = sqrt(pow((road_avr_intensity[0] - parnking_spot_avr_intensity[0]), 2) 
							+ pow((road_avr_intensity[1] - parnking_spot_avr_intensity[1]), 2) 
							+ pow((road_avr_intensity[2] - parnking_spot_avr_intensity[2]), 2));

		if (color_dist > param.parking_spot_color_difference)
			squares.erase(squares.begin() + i);
	}
}



vector<vector<Point> > parking_spot_detection(Mat &image_pers, double car_angle)
{

	if(car_angle > param.max_car_angle || car_angle < -param.max_car_angle){
		vector<vector<Point> > nothing;
		return nothing;
	}

  	Mat1b mask_seg;
  	Mat1b mask = segmentation_mask(param.pers_x, param.pers_y, 4, car_angle);
  	img_segmentation(image_pers, mask, mask_seg, param.grabCutIterations);

  	Scalar road_avr_intensity = mean(image_pers, mask_seg);
  	Mat image_seg(image_pers.size(), CV_8UC3, Scalar(road_avr_intensity));
  	image_pers.copyTo(image_seg, mask_seg);

  	Mat image_filtered;
    GaussianBlur(image_seg, image_filtered, Size(5,5), 6*param.gauss_sigma);
	
	vector<vector<Point> > squares;
    for(int i=0; i<3; i++){
		vector<vector<Point> > squares_temp;

        if(i==0){
            param.lowCan = 850;
            param.highCan = 900;
            param.dilatile1 = 4;
            param.dilatile2 = 7;
            param.erode1 = 4;
            param.erode2 = 4;
        }
        if(i==1){
          param.lowCan = 900;
          param.highCan = 1100;
          param.dilatile1 = 5;
          param.dilatile2 = 5;
          param.erode1 = 3;
          param.erode2 = 4;
        }
        if(i==2){
          param.lowCan = 1100;
          param.highCan = 1400;
          param.dilatile1 = 2;
          param.dilatile2 = 3;
          param.erode1 = 2;
          param.erode2 = 2;
        }


	  	Mat img_canny;
	  	Canny(image_filtered, img_canny, param.lowCan, param.highCan, param.canny_kernel_size);

	  	Mat img_canny_morph;
	  	dilate(img_canny, img_canny_morph, Mat(), Point(-1, -1), param.dilatile1);

	  	erode(img_canny_morph, img_canny_morph, Mat(), Point(-1, -1), param.erode1);

	  	dilate(img_canny_morph, img_canny_morph, Mat(), Point(-1, -1), param.dilatile2);

	  	erode(img_canny_morph, img_canny_morph, Mat(), Point(-1, -1), param.erode2);

		/*namedWindow("erode", WINDOW_NORMAL);
		imshow("erode", img_canny_morph);*/

	  	findSquares(img_canny_morph, squares_temp);
	  	sort_verticles(squares_temp);

		for(int j=0; j<squares_temp.size(); j++)
			squares.push_back(squares_temp[j]);
	}

  color_check(image_pers, squares, road_avr_intensity);
  lines_ratio_check(squares);
	eliminate_duplicates(squares);

	for(int i=0; i<squares.size(); i++){

		if(squares[i][0].y > 0.7*param.pers_y)
			squares.erase(squares.begin() + i);

	}

  /*drawSquares(image_pers, squares);
	namedWindow("Output", WINDOW_NORMAL);
	imshow("Output", image_pers);
  waitKey(10);*/


  return squares;
}

