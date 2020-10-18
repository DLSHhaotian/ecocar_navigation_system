#include <cstdio>
#include <ctime>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "ros/ros.h"

#define PATH "camera_images/"

using namespace std;
using namespace cv;

const double rate = 5; // in Hz
const int camera = 0;

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
    ros::init(argc, argv, "image_capturing");
    ros::start();
    ros::Rate loop_rate(rate);
    VideoCapture cap(camera);

    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 960);
    cap.set(CAP_PROP_FPS, 20);

    if(!cap.isOpened())
        return -1;

    Mat frame;
    if (cap.read(frame) == NULL)
        return 1;

    int i = 1;
    string start_time = currentDateTime();
    char name[100];

    cout << "Width using video.get(CAP_PROP_FRAME_WIDTH) : " << cap.get(CAP_PROP_FRAME_WIDTH ) << endl;
    cout << "Height using video.get(CAP_PROP_FRAME_HEIGHT) : " << cap.get(CAP_PROP_FRAME_HEIGHT ) << endl;
    cout << "Brightness using video.get(CAP_PROP_BRIGHTNESS) : " << cap.get(CAP_PROP_BRIGHTNESS ) << endl;
    cout << "Gain using video.get(CAP_PROP_GAIN) : " << cap.get(CAP_PROP_GAIN ) << endl;
    cout << "Frames per second using video.get(CAP_PROP_FPS) : " << cap.get(CAP_PROP_FPS) << endl;
    cout << "Exporure using video.get(CAP_PROP_EXPOSURE) : " << cap.get(CAP_PROP_EXPOSURE ) << endl;


    while(ros::ok())
    {
        if (cap.read(frame) == NULL)
            return 1;
        ros::Time current_time = ros::Time::now();
        // string current_time = currentDateTime();
        string path = PATH;
        sprintf(name, "%d-%d_%06d.jpg", (int) current_time.sec, (int) current_time.nsec/1000000, i++);
        imwrite(path + name, frame);
        ROS_INFO("Captured image %d", i);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


