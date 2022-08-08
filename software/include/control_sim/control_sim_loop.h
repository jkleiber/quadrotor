
#pragma once

#include "imgui.h"

#include "control_sim/control_loop.h"
#include "control_sim/events.h"
#include "control_sim/quadcopter_dynamics.h"
#include "control_sim/sim_clock.h"

class ControlSimLoop
{
public:
    ControlSimLoop(SimClock *clk, GuiEvents *gui_events)
        : quadcopter(clk), ctrl(clk), clk_(clk), gui_events_(gui_events)
    {
        InitSim();
    }

    bool InitSim();
    bool UpdateSim();
    bool RunSimLoop();

private:
    // Simulation state
    bool is_running;

    // Vehicle state
    Eigen::VectorXd x;
    Eigen::VectorXd u;

    // Dynamics and control
    QuadcopterDynamics quadcopter;
    ControlLoop ctrl;

    SimClock *const clk_;
    GuiEvents *const gui_events_;

    // Printing
    double prev_print_time;
};
