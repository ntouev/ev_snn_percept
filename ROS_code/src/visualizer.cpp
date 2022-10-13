#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <iostream>
#include <math.h>
#include <ctime>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "package_name/LinePolar.h" // <-- MODIFY!!

#include <opencv2/opencv.hpp>

#define PI 3.14159274101257324219

// ------------- USER INPUT -------------
#define RETINA_DIM 128
#define ZOOM 4 // visualization is a (128x4) x (128x4) window
#define FPS 30 // visualization FPS
// --------------------------------------

// Globals
struct Gmsg {
    float r;
    float theta;
    int t;
};
struct Gmsg gmsg = {0, 0, 0};

void det_pav_callback(const ev_hough::LinePolar::ConstPtr& msg)
{
    gmsg.r = msg->r;
    gmsg.theta = msg->theta;
    gmsg.t = msg->t;
}

void add_line_to_img(float r, float theta, cv::Mat img, int thickness)
{
    float x_0, x_1, y_0, y_1;

    if (theta >= 1.55) { // theta E [89, 90] degrees
        y_0 = r;
        y_1 = r;
        x_0 = 0;
        x_1 = RETINA_DIM - 1;
    } else if (theta <= -1.55) { // theta E [-89, -90] degrees
        y_0 = r;
        y_1 = r;
        x_0 = 0;
        x_1 = RETINA_DIM - 1;
    } else {
        y_0 = 0;
        y_1 = RETINA_DIM - 1;
        x_0 = (r - y_0*sin(theta))/cos(theta);
        x_1 = (r - y_1*sin(theta))/cos(theta);
    }

    cv::Point start_point(ZOOM*round(x_0), ZOOM*round(y_0)), end_point(ZOOM*round(x_1), ZOOM*round(y_1));
    cv::line(img, start_point, end_point, (0, 0, 0), thickness);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("det_pav", 1, det_pav_callback);

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Rate loop_rate(FPS);
    while (ros::ok())
    {
        ros::spinOnce();
        
        cv::Mat img(ZOOM*RETINA_DIM, ZOOM*RETINA_DIM, CV_8UC3, cv::Scalar(127, 127, 127));
        add_line_to_img(gmsg.r, gmsg.theta, img, 2);
        cv::imshow("real-time detected pavement", img);

        if (cv::waitKey(1) >= 0) {
            break;
        }

        loop_rate.sleep();
    }

    return 0;
}
