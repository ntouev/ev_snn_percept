#include "~/ev_snn_percept/Visualiser/spynnaker_external_device_lib/SpynnakerLiveSpikesConnection.h"
#include "~/ev_snn_percept/Visualiser/spynnaker_external_device_lib/WaitForStop.h"
#include "receiver_if.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <numeric>

int main(int argc, char **argv)
{
    try{
        ros::init(argc, argv, "live_output");

        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        //------------------------------------------------------------------------------------------------------------//
        char const* receive_label1 = "pop_output"; // must be the same as the name of the planar population of the SNN!
        char* receive_labels[1] = {(char *) receive_label1};
        char* send_labels[0];
        char const* local_host = NULL;
        std::mutex mtx;

        SpynnakerLiveSpikesConnection connection = SpynnakerLiveSpikesConnection(1, receive_labels, 0, send_labels, \
                                                                                 (char*)NULL, UDP_PORT);
        ROS_WARN("Listening on %d", connection.get_local_port());

        ReceiverInterface* receiver_callback = new ReceiverInterface(&mtx);
        connection.add_receive_callback((char*) receive_label1, receiver_callback);

        //------------------------------------------------------------------------------------------------------------//
        // Stop interface
        WaitForStop *wait_for_stop = new WaitForStop();
        connection.add_pause_stop_callback((char*) receive_label1, wait_for_stop);

        wait_for_stop->wait_for_stop();
        std::lock_guard<std::mutex> lock(mtx);
        
        //------------------------------------------------------------------------------------------------------------//
        ROS_WARN("Population %s received %d spikes", receive_label1, receiver_callback->get_n_spikes());
    }
    catch (std::exception &e){
        std::cout << "Exception caught: " << e.what() << "\n";
    }
    catch (...) {
        std::cout << "Unknown exception caught\n";
    }

  return 0;
}
