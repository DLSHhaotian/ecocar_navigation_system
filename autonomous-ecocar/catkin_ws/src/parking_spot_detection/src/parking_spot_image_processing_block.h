#ifndef PARKING_SPOT_IMAGE_PROCESSING_BLOCK_H
#define PARKING_SPOT_IMAGE_PROCESSING_BLOCK_H

#include <opencv2/opencv.hpp>

/* Parking spot block is returned as 4-Point long vectors. */
std::vector<std::vector<cv::Point2f> > parking_spot_detection_block(cv::Mat &image_pers, double car_angle, cv::Mat &H, int img_out_width, int img_out_height);

#endif // PARKING_SPOT_IMAGE_PROCESSING_BLOCK_H
