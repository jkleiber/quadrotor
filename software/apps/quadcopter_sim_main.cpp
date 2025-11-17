
#include <chrono>
#include <iostream>
#include <thread>

#include <quadcopter_simulator/quadcopter_simulator.h>

#include <Eigen/Dense>

int main(int argc, char **argv)
{
    // Timing
    double dt = 0.01;

    // Simulation clock
    SimClock clk(dt);

    // Control simulation loop
    QuadcopterSim sim(&clk);

    bool is_running = true;
    while (is_running)
    {
        // Update the simulation.
        sim.Update();
    }

    return 0;
}