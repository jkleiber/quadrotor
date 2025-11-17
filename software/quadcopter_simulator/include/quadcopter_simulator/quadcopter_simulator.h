
#pragma once

#include "quadcopter_simulator/logging.h"
#include "quadcopter_simulator/math_utils.h"
#include "quadcopter_simulator/sim_clock.h"

#include <quadcopter/dynamics.h>
#include <quadcopter/parameters.h>

typedef struct sim_state_t
{
    double altitude = 0.0;
} SimState;

class QuadcopterSim
{

public:
    QuadcopterSim(SimClock &clk)
        : quadcopter_(clk.GetDt()), clk_(clk), state_log_(clk)
    {
        Init();
    }

    void Init();
    void Update();

    void ApplyMotorControl(const double &front_left, const double &front_right,
                           const double &back_left, const double &back_right);

    SimState GetSimState() const;

private:
    // Dynamics and control
    quadcopter::Dynamics quadcopter_;
    Eigen::VectorXd motor_throttles_;

    SimClock &clk_;

    // Sim state.
    SimState sim_state_;
    void UpdateSimState();

    // Printing
    double prev_print_time_;

    // Logging.
    Logging state_log_;
    void InitLogging();
    void LogState();
};
