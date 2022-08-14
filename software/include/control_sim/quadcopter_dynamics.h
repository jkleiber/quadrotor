#ifndef QUADCOPTER_DYNAMICS_H
#define QUADCOPTER_DYNAMICS_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <random>
#include <string>

#include "imgui.h"

#include "control_sim/logging.h"
#include "control_sim/sim_clock.h"

class QuadcopterDynamics
{
public:
    QuadcopterDynamics(SimClock *clk)
        : motor_dist(0, 0.5), state_log(clk), clk_(clk)
    {
        this->init(Eigen::VectorXd::Zero(12));
    }

    QuadcopterDynamics(Eigen::VectorXd x0, SimClock *clk)
        : motor_dist(0, 0.5), state_log(clk), clk_(clk), is_default(true)
    {
        this->init(x0);
    }

    void init(Eigen::VectorXd x0);

    // Compute the forces for each motor
    void get_motor_forces(Eigen::VectorXd u);

    // Update the dynamics (discrete)
    void update_dynamics(Eigen::VectorXd u0);

    bool UpdateParams(bool is_enabled);

    // Get the current quadcopter state
    Eigen::VectorXd get_state();

    // Convert state to string
    std::string state_to_string(Eigen::VectorXd x);

private:
    // States
    Eigen::VectorXd x;
    Eigen::VectorXd x_motors;
    Eigen::VectorXd u_prev;

    // Discrete time update
    double dt_;

    // Mass (kg)
    double m;

    // Gravity (m/s^2)
    double g;

    // Force and moment constants
    double K_f; // Force
    double K_m; // Moment
    double Kf_divisor_; // Divide 

    // Inertia (kg / m^2)
    double Ixx;
    double Ixy;
    double Ixz;
    double Iyy;
    double Iyz;
    double Izz;

    // Dimensions (m)
    double dx_arm;
    double dy_arm;

    // Limits
    double max_motor_force; // Max motor force (N)
    double max_omega;       // Max angular velocity (rad / s)
    double motor_slew_rate; // Max motor slew rate (% / s)

    // Disturbances
    std::default_random_engine motor_gen;
    std::normal_distribution<double> motor_dist;

    // State logging
    Logging state_log;

    SimClock *const clk_;

    // Parameter management
    bool is_default;
    std::ofstream quadcopter_param_file;

    // Vehicle parameters
    void LoadDefaultParams();
    void SetVehicleParams();
    void SaveParams();
};

#endif