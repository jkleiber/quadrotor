#ifndef QUADCOPTER_DYNAMICS_H
#define QUADCOPTER_DYNAMICS_H

#include <fstream>
#include <iostream>
#include <math.h>
#include <random>
#include <string>

#include <Eigen/Dense>
#include "imgui.h"

#include "control_sim/logging.h"
#include "control_sim/sim_clock.h"

class QuadcopterDynamics
{
public:
    QuadcopterDynamics(SimClock *clk)
        : state_log(clk), clk_(clk), is_default(true)
    {
        this->Init(Eigen::VectorXd::Zero(12));
    }

    QuadcopterDynamics(Eigen::VectorXd x0, SimClock *clk)
        : state_log(clk), clk_(clk), is_default(true)
    {
        this->Init(x0);
    }

    void Init(Eigen::VectorXd x0);

    // Compute the forces for each motor
    void GetMotorForces(Eigen::VectorXd u);

    // Update the dynamics (discrete)
    void UpdateDynamics(Eigen::VectorXd u0);

    // Allow for the GUI to update parameters
    void IdleLoop();
    bool UpdateParams(bool is_enabled);

    // Get the current quadcopter state
    Eigen::VectorXd GetState();

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
    double dist_mean;
    double dist_stddev;

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