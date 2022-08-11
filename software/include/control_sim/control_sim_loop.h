
#pragma once

#include "imgui.h"

#include "control_sim/control_loop.h"
#include "control_sim/events.h"
#include "control_sim/math_utils.h"
#include "control_sim/plotting.h"
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
    bool View();
    bool ResetPlots();

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

    // Plotting
    Plotting::ScrollingBuffer roll_chart;
    Plotting::ScrollingBuffer pitch_chart;
    Plotting::ScrollingBuffer yaw_chart;
};
