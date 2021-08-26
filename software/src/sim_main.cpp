// System Libraries
#include <iostream>
#include <unistd.h>

// Gazebo
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

int main(int argc, char **argv)
{
    // Load gazebo as a client
    gazebo::client::setup(argc, argv);

    // Run robot code
    while(true)
    {
        usleep(100000);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}