
#include <chrono>
#include <iostream>
#include <thread>

#include "control_sim/control_sim_gui.h"
#include "control_sim/control_sim_loop.h"
#include "control_sim/events.h"

#include "SDL.h"
#include <Eigen/Dense>

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
        // Create the frame and set some static GUI components.
        is_running &= gui_.UpdateGui();
        if (!is_running)
            std::cout << "update gui: " << is_running << std::endl;

        // Update the simulation.
        is_running &= sim_loop.UpdateSim();
        if (!is_running)
            std::cout << "update sim: " << is_running << std::endl;

        // Render at the end of the loop so other functions can update the GUI.
        is_running &= gui_.Render();
        if (!is_running)
            std::cout << "render: " << is_running << std::endl;
    }

    return 0;
}