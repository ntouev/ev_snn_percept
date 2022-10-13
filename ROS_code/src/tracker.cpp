#include <stdio.h>
#include <unistd.h>
#include <mutex>
#include <iostream>
#include <math.h>
#include <ctime>

#include "ros/ros.h"
#include "package_name/Spike.h"     // <-- MODIFY!!
#include "package_name/LinePolar.h" // <-- MODIFY!!

#define PI 3.14159274101257324219

// ------------- USER INPUT -------------
#define T_HIDDEN   50
#define T_VISIBLE  10000
#define D_THREAS   10
#define ALPHA      0.7
#define K          100

// apply (or not) initial detected contour r, theta values
#define REAL_DETECTION
#if not defined(REAL_DETECTION)
    #define ON_R_INIT      34
    #define ON_THETA_INIT  43
    #define OFF_R_INIT     34
    #define OFF_THETA_INIT 43
#endif

#define RETINA_DIM 128
#define X_NUM       32
#define THETA_NUM 90
#define R_NUM 60
// --------------------------------------

// do not change
#define HORIZ_THREAS -1

// Globals
ev_hough::LinePolar g_det_pav_msg;

struct Cluster {
    int id;
    int t;
    float r;
    float theta;
    int t_exp;
    int n;
};

struct Pavement {
    bool detected;
    float r;
    float theta;
    int ON_id;
    int OFF_id;
};
struct Pavement pavement;

int id = -1;
std::mutex *mtx;
std::list<struct Cluster> ON_cluster_list;
std::list<struct Cluster> OFF_cluster_list;

bool new_message = false;

void quant_to_real_128(float &r, float &theta) 
{
    r = (r - float(R_NUM)/2)*(2*float(X_NUM)*std::sqrt(2)/float(R_NUM));
    theta = (theta - float(THETA_NUM)/2)*(PI/float(THETA_NUM));

    r = float(RETINA_DIM)/float(X_NUM) * r;
}

void initialize_pavement(void)
{
    pavement = {false, 0, 0, 0, 0};

#if not defined(REAL_DETECTION)
    struct Cluster init_cluster;
    init_cluster = {
        -444,
        0,
        ON_R_INIT,
        ON_THETA_INIT,
        0 + T_VISIBLE,
        K
    };
    ON_cluster_list.push_back(init_cluster);
    
    init_cluster = {
        -444,
        0,
        OFF_R_INIT,
        OFF_THETA_INIT,
        0 + T_VISIBLE,
        K
    };
    OFF_cluster_list.push_back(init_cluster);
#endif
}

void spike_callback(const ev_hough::Spike::ConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();
 
    new_message = true;

    struct Cluster cluster;

    if (pavement.detected == false) {
        ROS_DEBUG(".");
    }
    
    for (int neuron_id_position = 0; neuron_id_position < msg->neuron_id.size(); neuron_id_position++) {
        if (msg->neuron_id[neuron_id_position] < THETA_NUM*R_NUM) {
        // ------------------------------ ON EVENTS -------------------------------- //
            float r = std::floor(msg->neuron_id[neuron_id_position]/THETA_NUM);
            float theta = msg->neuron_id[neuron_id_position]%THETA_NUM;

            // deals with 0 and THETA_NUM actually referring to lines with same angle (-pi/2 and pi/2)
            // mathematicaly this means theta E (-pi/2, pi/2]
            if (theta <= HORIZ_THREAS) {
                theta = THETA_NUM;
                r = 2*std::abs(R_NUM/2 - r) + r;
            }
                
            if (pavement.detected == false) {
                for (auto it = ON_cluster_list.begin(); it != ON_cluster_list.end();) {
                    if (msg->t > (*it).t_exp) {
                        it = ON_cluster_list.erase(it);
                    }
                    else {
                        it ++;
                    }
                }
            }

            if (ON_cluster_list.size() != 0) {
                float d_min = std::numeric_limits<float>::max();
                int id_min = -1; // initialization
                for (auto it = ON_cluster_list.begin(); it != ON_cluster_list.end(); it++) {
                    float d = std::sqrt(((*it).r - r)*((*it).r - r) + ((*it).theta - theta)*((*it).theta - theta));
                    if (d < d_min) {
                        d_min = d;
                        id_min = (*it).id;
                    }
                }

                if (d_min <= D_THREAS) {
                    for (auto it = ON_cluster_list.begin(); it != ON_cluster_list.end(); it++) {
                        if ((*it).id == id_min) {
                            // found the desired cluster!
                            (*it).r     = (1 - ALPHA)*(*it).r     + ALPHA*r;
                            (*it).theta = (1 - ALPHA)*(*it).theta + ALPHA*theta;

                            if ((*it).n >= K - 1) {
                                (*it).n = K;
                                (*it).t_exp = msg->t + T_VISIBLE;
                            }
                            else {
                                (*it).n ++;
                                (*it).t_exp = msg->t + T_HIDDEN;
                            }
                        }
                    }
                }
                else {
                    if (pavement.detected == false) {
                        id ++;
                        cluster = {
                            id,
                            msg->t,
                            r,
                            theta,
                            msg->t + T_HIDDEN,
                            0
                        };
                        ON_cluster_list.push_back(cluster);
                    }               
                }
            }
            else {
                if (pavement.detected == false) {
                    id ++;
                    cluster = {
                        id,
                        msg->t,
                        r,
                        theta,
                        msg->t + T_HIDDEN,
                        0
                    };
                    ON_cluster_list.push_back(cluster);
                }
            }
        }
        else {
            // ------------------------------ OFF EVENTS -------------------------------- //
            float r = std::floor(msg->neuron_id[neuron_id_position]/THETA_NUM) - R_NUM;
            float theta = msg->neuron_id[neuron_id_position]%THETA_NUM;
            
            if (theta <= HORIZ_THREAS) {
                theta = THETA_NUM;
                r = 2*std::abs(R_NUM/2 - r) + r;
            }

            if (pavement.detected == false) {
                for (auto it = OFF_cluster_list.begin(); it != OFF_cluster_list.end();) {
                    if (msg->t > (*it).t_exp) {
                        it = OFF_cluster_list.erase(it);
                    }
                    else {
                        it ++;
                    }
                }
            }
                
            if (OFF_cluster_list.size() != 0) {
                float d_min = std::numeric_limits<float>::max(); 
                int id_min = -1; // initialization
                for (auto it = OFF_cluster_list.begin(); it != OFF_cluster_list.end(); it++) {
                    float d = std::sqrt(((*it).r - r)*((*it).r - r) + ((*it).theta - theta)*((*it).theta - theta));
                    if (d < d_min) {
                        d_min = d;
                        id_min = (*it).id;
                    }
                }

                if (d_min <= D_THREAS) {
                    for (auto it = OFF_cluster_list.begin(); it != OFF_cluster_list.end(); it++) {
                        if ((*it).id == id_min) {
                            // found the desired cluster!
                            (*it).r     = (1 - ALPHA)*(*it).r     + ALPHA*r;
                            (*it).theta = (1 - ALPHA)*(*it).theta + ALPHA*theta;

                            if ((*it).n >= K - 1) {
                                (*it).n = K;
                                (*it).t_exp = msg->t + T_VISIBLE;
                            }
                            else {
                                (*it).n ++;
                                (*it).t_exp = msg->t + T_HIDDEN;
                            }
                        }
                    }
                }
                else {
                    if (pavement.detected == false) {
                        id ++;
                        cluster = {
                            id,
                            msg->t,
                            r,
                            theta,
                            msg->t + T_HIDDEN,
                            0
                        };
                        OFF_cluster_list.push_back(cluster);
                    }               
                }
            }
            else {
                if (pavement.detected == false) {
                    id ++;
                    cluster = {
                        id,
                        msg->t,
                        r,
                        theta,
                        msg->t + T_HIDDEN,
                        0
                    };
                    OFF_cluster_list.push_back(cluster);
                }
            }
        }
       

        for (auto it = ON_cluster_list.begin(); it != ON_cluster_list.end(); it++) {
            (*it).t = msg->t;
        }
        for (auto it = OFF_cluster_list.begin(); it != OFF_cluster_list.end(); it++) {
            (*it).t = msg->t;
        }

        // Higher level detection (pavement)
        if (pavement.detected == false) {
            // the logic below assumes that the first detected pair of ON-OFF 
            // clusters is the desired pavement
            for (auto it_on = ON_cluster_list.begin(); it_on != ON_cluster_list.end(); it_on++) {
                if ((*it_on).n == K) {
                    for (auto it_off = OFF_cluster_list.begin(); it_off != OFF_cluster_list.end(); it_off++) {
                        if ((*it_off).n == K) {
                            pavement.detected = true;
                            pavement.ON_id = (*it_on).id;
                            pavement.OFF_id = (*it_off).id; 
                        }
                    }
                }
            }
            if (pavement.detected == true) {
                ROS_WARN("Pavement detected: ON_id = %d, OFF_id = %d", pavement.ON_id, pavement.OFF_id);
            }
        }

        if (pavement.detected == true) {
                for (auto it_on = ON_cluster_list.begin(); it_on != ON_cluster_list.end(); it_on++) {
                    if ((*it_on).id == pavement.ON_id) {
                        for (auto it_off = OFF_cluster_list.begin(); it_off != OFF_cluster_list.end(); it_off++) {
                            if ((*it_off).id == pavement.OFF_id) {
                                pavement.r = ((*it_on).r + (*it_off).r)/2;
                                pavement.theta = ((*it_on).theta + (*it_off).theta)/2;
                            }
                        }
                    }
                }

            // Publicing ...
            quant_to_real_128(pavement.r, pavement.theta);

            g_det_pav_msg.r = pavement.r;
            g_det_pav_msg.theta = pavement.theta;
            g_det_pav_msg.t = msg->t;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    if (elapsed.count() >= 1000) {
        ROS_WARN("Exceeded 1000 usec! --> %ld usec <-- !!!", elapsed.count());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    
    ros::NodeHandle n;
    ros::Publisher  det_pav_pub = n.advertise<ev_hough::LinePolar>("det_pav", 1);
    ros::Subscriber spike_sub = n.subscribe("spike", 1, spike_callback);

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    initialize_pavement();

    while (ros::ok())
    {
        ros::spinOnce();

        if (new_message) {
            det_pav_pub.publish(g_det_pav_msg);
            new_message = false;
        }
    }

    return 0;
}
