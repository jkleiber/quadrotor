
#pragma once

#include "imgui.h"

#include "control_sim/control_loop.h"
#include "control_sim/events.h"
#include "control_sim/math_utils.h"
#include "control_sim/plotting.h"
#include "control_sim/sim_clock.h"

#include <quadcopter/dynamics.h>
#include <quadcopter/parameters.h>

class ControlSimLoop
{
public:
    ControlSimLoop(SimClock *clk, GuiEvents *gui_events)
        : quadcopter(clk->GetDt()), ctrl(clk), clk_(clk),
          gui_events_(gui_events), state_log_(clk)
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
    bool is_initial;

    // Vehicle state
    Eigen::VectorXd x;
    Eigen::VectorXd u;

    // Dynamics and control
    quadcopter::Dynamics quadcopter;
    ControlLoop ctrl;

    SimClock *const clk_;
    GuiEvents *const gui_events_;

    // Printing
    double prev_print_time;

    // Plotting
    Plotting::DynamicGraph xy_chart;
    Plotting::ScrollingBuffer height_chart;
    Plotting::ScrollingBuffer roll_chart;
    Plotting::ScrollingBuffer pitch_chart;
    Plotting::ScrollingBuffer yaw_chart;

    // Parameter editing
    void UpdateQuadcopterParams(bool is_enabled);

    // Logging.
    Logging state_log_;
    void InitLogging();
    void LogState();
};
