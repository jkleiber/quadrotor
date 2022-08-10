
#include <chrono>
#include <iostream>
#include <thread>

#include "control_sim/control_sim_gui.h"
#include "control_sim/control_sim_loop.h"
#include "control_sim/events.h"

#include <Eigen/Dense>
#include "SDL.h"

int main(int argc, char **argv)
{
    // Timing
    double dt = 0.01;

    // Simulation clock
    SimClock clk(dt);

    // Shared memory
    GuiEvents gui_events;

    // Control simulation loop
    ControlSimLoop sim_loop(&clk, &gui_events);

    // GUI
    GuiManager gui_(&gui_events);
    gui_.InitGui();

    bool is_running = true;
    while (is_running)
    {
        is_running &= gui_.UpdateGui();
        is_running &= sim_loop.UpdateSim();
    }

    return 0;
}