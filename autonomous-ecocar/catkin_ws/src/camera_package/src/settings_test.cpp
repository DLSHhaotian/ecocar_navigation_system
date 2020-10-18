#include <cstdio>
#include <ctime>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "ros/ros.h"

#define PATH "camera_images/"

using namespace std;
using namespace cv;

const double rate = 0.5; // in Hz
const int camera = 1;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80]; 
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%X", &tstruct);

    return buf;
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "settings_test");
    ros::start();

    // Start default camera
    VideoCapture video(camera);

    video.set(CAP_PROP_FRAME_WIDTH, 640);
    video.set(CAP_PROP_FRAME_HEIGHT, 480);
    video.set(CAP_PROP_FPS, 30);

    // video.set(CAP_PROP_EXPOSURE, 10);
    // video.set(CAP_PROP_BRIGHTNESS, 1.0);
    // video.set(CAP_PROP_GAIN, 1.0);
     
    // With webcam get(CV_CAP_PROP_FPS) does not work.
    // Let's see for ourselves.
     
    double fps = video.get(CV_CAP_PROP_FPS);
    // If you do not care about backward compatibility
    // You can use the following instead for OpenCV 3
    // double fps = video.get(CAP_PROP_FPS);
    cout << "Width using video.get(CAP_PROP_FRAME_WIDTH) : " << video.get(CAP_PROP_FRAME_WIDTH ) << endl;
    cout << "Height using video.get(CAP_PROP_FRAME_HEIGHT) : " << video.get(CAP_PROP_FRAME_HEIGHT ) << endl;
    cout << "Brightness using video.get(CAP_PROP_BRIGHTNESS) : " << video.get(CAP_PROP_BRIGHTNESS ) << endl;
    cout << "Gain using video.get(CAP_PROP_GAIN) : " << video.get(CAP_PROP_GAIN ) << endl;
    cout << "Frames per second using video.get(CAP_PROP_FPS) : " << fps << endl;
    cout << "Exporure using video.get(CAP_PROP_EXPOSURE) : " << video.get(CAP_PROP_EXPOSURE ) << endl;
     
 
    // Number of frames to capture
    int num_frames = 1000;
     
    // Start and end times
    time_t start, end;
     
    // Variable for storing video frames
    Mat frame;
 
    cout << "Capturing " << num_frames << " frames" << endl ;
 
    video >> frame;

    // Start time
    time(&start);
     
    // Grab a few frames
    for(int i = 0; i < num_frames; i++)
    {
        video >> frame;
        // cout << "Gain using video.get(CAP_PROP_GAIN) : " << video.get(CAP_PROP_GAIN ) << endl;
    }
     
    // End Time
    time(&end);
     
    // Time elapsed
    double seconds = difftime (end, start);
    cout << "Time taken : " << seconds << " seconds" << endl;
     
    // Calculate frames per second
    fps  = num_frames / seconds;
    cout << "Estimated frames per second : " << fps << endl;
     
    // Release video
    video.release();
    return 0;
}


