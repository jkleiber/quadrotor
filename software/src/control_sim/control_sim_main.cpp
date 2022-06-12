
#include <iostream>

#include "control_sim/control_loop.h"
#include "control_sim/quadcopter_dynamics.h"


#include <eigen3/Eigen/Dense>


int main(int argc, char **argv)
{
    std::cout << "Control Sim\n";

    // Configuration
    int max_iter = 10000; // 100 sec
    int print_every = 100; // 1 sec
    double dt = 0.01;
    double t = 0;

    // Setpoints
    // Maintain level flight while taking off
    Eigen::VectorXd setpoints = Eigen::VectorXd::Zero(12);

    // Initial Conditions
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);

    // State variables
    Eigen::VectorXd x = x0;

    // Control
    Eigen::VectorXd u = Eigen::VectorXd::Zero(4);

    // Initialize Quadcopter dynamics
    QuadcopterDynamics quadcopter(x0, dt);

    // Initialize control loop
    ControlLoop ctrl;
    

    // Simulation loop
    for (int i = 0; i < max_iter; ++i)
    {
        // Control loop
        u = ctrl.run_loop(x, setpoints);

        // Quadcopter dynamics
        quadcopter.update_dynamics(u);

        // Get state variables
        x = quadcopter.get_state();

        // Print status at a certain frequency
        if (i%print_every == 0)
        {
            std::cout << "t=" << t << "sec" << std::endl;
        }

        // Increment time
        t += dt;
    }

    // TODO: export x trajectory to csv for visualization


    return 0;
}