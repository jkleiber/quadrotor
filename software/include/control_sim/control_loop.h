#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Eigen/Dense>

#include "imgui.h"

#include "control_sim/events.h"
#include "control_sim/gains.h"
#include "control_sim/logging.h"
#include "control_sim/pid_controller.h"
#include "control_sim/sim_clock.h"

class ControlLoop
{
public:
    ControlLoop(SimClock *clk)
        : clk_(clk), height_pid(clk), roll_pid(clk), pitch_pid(clk),
          yaw_pid(clk), ctrl_log(clk), is_default(true)
    {
        Init();
    }

    // Initialize
    void Init();

    // Given state and targets, control the quadcopter
    Eigen::VectorXd RunLoop(Eigen::VectorXd x, Eigen::VectorXd setpoints);
    
    // Run this when the simulation is reset to update the gains.
    void IdleLoop();

    bool UpdateParams(bool enabled);

private:
    // Timing
    SimClock const *clk_;

    // Controllers
    PIDController height_pid;
    PIDController roll_pid;
    PIDController pitch_pid;
    PIDController yaw_pid;

    // Feedforward on height PID.
    // This is the operating throttle to maintain constant height
    double height_pid_ff;

    // Logging
    Logging ctrl_log;

    // PID Gains
    PIDGains height_gains;
    PIDGains roll_gains;
    PIDGains pitch_gains;
    PIDGains yaw_gains;

    // Save gains to file until loading is better.
    std::ofstream gain_file;

    // Gain default tracking
    bool is_default;

    // Gain management functions
    void SetDefaultGains();
    void ApplyGains();
    void SaveGains();
};

#endif