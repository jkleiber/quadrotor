#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <eigen3/Eigen/Dense>

#include "control_sim/pid_controller.h"

class ControlLoop
{
    public:
        ControlLoop();

        // Given state and targets, control the quadcopter
        Eigen::VectorXd run_loop(Eigen::VectorXd x, Eigen::VectorXd setpoints);

    private:
        PIDController height_pid;

        // Feedforward on height PID.
        // This is the operating throttle to maintain constant height
        double height_pid_ff; 
};

#endif