#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <eigen3/Eigen/Dense>

#include "control_sim/sim_clock.h"
#include "control_sim/pid_controller.h"
#include "control_sim/logging.h"

class ControlLoop
{
    public:
        ControlLoop(SimClock *clk) 
            : clk_(clk), height_pid(clk), roll_pid(clk), pitch_pid(clk), yaw_pid(clk), ctrl_log("ctrl_log.csv", clk)
        {
            init();
        }

        // Initialize
        void init();

        // Given state and targets, control the quadcopter
        Eigen::VectorXd run_loop(Eigen::VectorXd x, Eigen::VectorXd setpoints);

    private:
        // Timing
        SimClock const* clk_;

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
};

#endif