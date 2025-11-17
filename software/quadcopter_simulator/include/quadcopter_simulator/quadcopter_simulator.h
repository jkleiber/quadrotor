
#pragma once

#include "quadcopter_simulator/logging.h"
#include "quadcopter_simulator/math_utils.h"
#include "quadcopter_simulator/sim_clock.h"

#include <quadcopter/dynamics.h>
#include <quadcopter/parameters.h>

class QuadcopterSim
{
public:
    QuadcopterSim(SimClock *clk)
        : quadcopter_(clk->GetDt()), clk_(clk), state_log_(clk)
    {
        Init();
    }

    void Init();
    void Update();

private:
    // Dynamics and control
    quadcopter::Dynamics quadcopter_;

    SimClock *const clk_;

    // Printing
    double prev_print_time_;

    // Logging.
    Logging state_log_;
    void InitLogging();
    void LogState();
};
