#ifndef RECEIVER_IF_H
#define RECEIVER_IF_H

#include "~/ev_snn_percept/Visualiser/spynnaker_external_device_lib/SpynnakerLiveSpikesConnection.h"
#include <mutex>
#include <list>

#include "ros/ros.h"
#include "package_name/Spike.h" // <-- MODIFY!!

// must be the same with the udp port defined in spinnaker/scripts/rt-online_ht_snn.py script!
#define UDP_PORT 56789

// Class 
class ReceiverInterface : public SpikeReceiveCallbackInterface {
public:
    ReceiverInterface(std::mutex *mtx) : mtx(mtx){};
    
    void receive_spikes(char *label, int time, int n_spikes, int* spikes);
    
    int get_n_spikes() {
        return this->n_spikes;
    }

private:
    ros::NodeHandle n;
    ros::Publisher spike_pub = n.advertise<ev_hough::Spike>("spike", 1);

    int n_spikes = 0;
    std::mutex *mtx;
};

#endif // RECEIVER_IF_H