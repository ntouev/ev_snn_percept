#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <iostream>
#include <math.h>
#include <ctime>

#include "ros/ros.h"
#include "package_name/LinePolar.h" // <-- MODIFY!!
#include "package_name/Feature.h"   // <-- MODIFY!!


// ------------- USER INPUT -------------
#define RETINA_DIM 128

// the frequency of the feature publishing. Used in case a synchronous 
// control module is applied. In case of an asynchronous controller 
// remove the relevant code and let the feature publishing be asynchronous
#define FREQ 30 

// C = height/2 of the trapezoid bounding the detected area
#define C 5
// --------------------------------------

// Globals
ev_hough::Feature g_feat_msg;
int y_axis[2] = {0, RETINA_DIM - 1};
int x_axis[2] = {0, RETINA_DIM - 1};
std::pair<int, int> p1_ref(RETINA_DIM/2 + C, 127);
std::pair<int, int> p2_ref(RETINA_DIM/2 - C, 127);
std::pair<int, int> p3_ref(RETINA_DIM/2 - C, 0);
std::pair<int, int> p4_ref(RETINA_DIM/2 + C, 0);

bool new_message = false;

void det_pav_callback(const ev_hough::LinePolar::ConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();

    new_message = true;

    // avoid divisions with 0
    if ((sin(msg->theta) == 0) || (cos(msg->theta) == 0))
        return;

    float r[2] = {msg->r - C,  msg->r + C};

    std::vector<std::pair<int, int>> points_vector;
    std::vector<std::vector<std::pair<int, int>>> best_fits;
    std::vector<std::vector<int>> distances;
    std::vector<int> feature;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) { 

            float x_temp = (r[i] - y_axis[j]*sin(msg->theta))/cos(msg->theta); 
            if ((x_temp >= 0) && (x_temp <= RETINA_DIM - 1)) {
                std::pair<int, int> p(int(std::round(x_temp)), y_axis[j]);
                points_vector.push_back(p);
            }
            
            float y_temp = (r[i] - x_axis[j]*cos(msg->theta))/sin(msg->theta);
            if ((y_temp >= 0) && (y_temp <= RETINA_DIM - 1)) {
                std::pair<int, int> p(x_axis[j], int(std::round(y_temp)));
                points_vector.push_back(p);
            }
        }
    }

    // avoid wrong features, if detected line is to close to the frame boundaries
    if (points_vector.size() != 4) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (j != i) {
                for (int k = 0; k < 4; k++) {
                    if ((k != i) && (k != j)) {
                        for ( int l = 0; l < 4; l++) {
                            if ((l != i) && (l != j) && (l != k)) {
                                std::pair<int, int> a = points_vector[i];
                                std::pair<int, int> b = points_vector[j];
                                std::pair<int, int> c = points_vector[k];
                                std::pair<int, int> d = points_vector[l];
                            
                                int dist = (a.first - p1_ref.first) * (a.first - p1_ref.first) + (a.second - p1_ref.second) * (a.second - p1_ref.second) + 
                                           (b.first - p2_ref.first) * (b.first - p2_ref.first) + (b.second - p2_ref.second) * (b.second - p2_ref.second) + 
                                           (c.first - p3_ref.first) * (c.first - p3_ref.first) + (c.second - p3_ref.second) * (c.second - p3_ref.second) + 
                                           (d.first - p4_ref.first) * (d.first - p4_ref.first) + (d.second - p4_ref.second) * (d.second - p4_ref.second);
                                std::vector<int> distance = {dist, i, j, k, l};
                                distances.push_back(distance);
                            }
                        }
                    }
                }
            }
        }
    }

    int min_dist = 999999;
    for (auto it = distances.begin(); it != distances.end(); it++) {
        int dist = (*it)[0];
        if (dist < min_dist) {
            min_dist = dist;
        }
    }

    for (auto it = distances.begin(); it != distances.end(); it++) {
        int dist = (*it)[0];
        
        if (dist == min_dist) {
            int i = (*it)[1];
            int j = (*it)[2];
            int k = (*it)[3];
            int l = (*it)[4];

            std::vector<std::pair<int, int>> best_fit = {points_vector[i], points_vector[j], points_vector[k], points_vector[l]};
            best_fits.push_back(best_fit);
        }
    }

    int A_max = -1;
    for (auto it = best_fits.begin(); it != best_fits.end(); it++) {
        std::pair<int, int> a = (*it)[0];
        std::pair<int, int> b = (*it)[1];
        std::pair<int, int> c = (*it)[2];
        std::pair<int, int> d = (*it)[3];

        int A = (a.first * b.second - b.first * a.second + 
                 b.first * c.second - c.first * b.second + 
                 c.first * d.second - d.first * c.second +     
                 d.first * a.second - a.first * d.second);
        if (A > A_max) {
            A_max = A;
            feature = {a.first, a.second, b.first, b.second, c.first, c.second, d.first, d.second};
        }
    }

    g_feat_msg.p1x = feature[0];
    g_feat_msg.p1y = feature[1];
    g_feat_msg.p2x = feature[2];
    g_feat_msg.p2y = feature[3];
    g_feat_msg.p3x = feature[4];
    g_feat_msg.p3y = feature[5];
    g_feat_msg.p4x = feature[6];
    g_feat_msg.p4y = feature[7];
    g_feat_msg.r     = msg->r;
    g_feat_msg.theta = msg->theta;
    g_feat_msg.t     = msg->t;

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    // ROS_DEBUG("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\t%f\t%d\t%ld", feature[0], feature[1], feature[2], feature[3], \
    //                                                              feature[4], feature[5], feature[6], feature[7], \
    //                                                              msg->r, msg->theta, msg->t, elapsed.count());

    if (elapsed.count() >= 1000) {
        ROS_WARN("Exceeded 1000 usec! --> %ld usec <-- !!!", elapsed.count());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "featurer");

    ros::NodeHandle n;
    ros::Publisher  feat_pub = n.advertise<ev_hough::Feature>("feat", 1);
    ros::Subscriber det_pav_sub = n.subscribe("det_pav", 1, det_pav_callback);

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::Rate loop_rate(FREQ);
    while (ros::ok())
    {
        ros::spinOnce();

        if (new_message) {
            feat_pub.publish(g_feat_msg);
            new_message = false;
        }

        loop_rate.sleep();
    }

    return 0;
}
