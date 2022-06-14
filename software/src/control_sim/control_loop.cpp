#include "control_sim/control_loop.h"

ControlLoop::ControlLoop()
{
    // Init Height PID
    // NOTE: Height PID simulates pilot flying ability for now
    height_pid.begin(0, 0.175, 0.001, 0.05);
    height_pid.setOutputRange(-0.25, 0.25);

    // Set feedforward to maintain constant height
    height_pid_ff = 0.565;
}

Eigen::VectorXd ControlLoop::run_loop(Eigen::VectorXd x, Eigen::VectorXd setpoints)
{
    // Setpoints correspond to states directly
    Eigen::VectorXd error = setpoints - x;

    double height = x(8);
    double height_target = setpoints(8);

    double throttle = height_pid_ff + height_pid.update(height_target, height);
    throttle = std::min(1.0, std::max(0.0, throttle)); // Ensure between 0 and 1

    // Set throttle
    return throttle * Eigen::VectorXd::Ones(4);
}