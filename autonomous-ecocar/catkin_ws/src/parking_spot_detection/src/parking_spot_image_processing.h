#ifndef PARKING_SPOT_IMAGE_PROCESSING_H
#define PARKING_SPOT_IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>

/* Parking spot corners are returned as 4-Point long vectors. All detected
 * spots are contained within outer vector. */
std::vector<std::vector<cv::Point> > parking_spot_detection(cv::Mat &image_pers, double car_angle);

#endif // PARKING_SPOT_IMAGE_PROCESSING_H
