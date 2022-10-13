#include "receiver_if.h"
#include <stdio.h>
#include <chrono>
#include <ctime>
#include <cmath>
#include <iostream>
#include <algorithm>

void ReceiverInterface::receive_spikes(char *label, int time, int n_spikes, int* spikes) 
{   
    std::lock_guard<std::mutex> lock(*mtx);
    
    auto start = std::chrono::high_resolution_clock::now();

    ev_hough::Spike spike_msg;
    std::vector<int> neurons_vector;

    // as long as spikes are received from the planar population print dots just for visual feedback
    std::stringstream ss;
    for (int neuron_id_position = 0;  neuron_id_position < n_spikes; neuron_id_position++) {
        neurons_vector.push_back(spikes[neuron_id_position]);
        ss << "...";
    }
    std::cout << ss.str() << std::endl;
    ss.clear();

    spike_msg.neuron_id = neurons_vector;
    spike_msg.t = time;
    spike_pub.publish(spike_msg);
    
    this->n_spikes += n_spikes;
    
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    if (elapsed.count() >= 1000) {
        ROS_WARN("Exceeded 1000 usec! --> %ld usec <-- !!!", elapsed.count());
    }
}
