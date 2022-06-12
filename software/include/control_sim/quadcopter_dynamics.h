#ifndef QUADCOPTER_DYNAMICS_H
#define QUADCOPTER_DYNAMICS_H

#include <math.h>
#include <eigen3/Eigen/Dense>


class QuadcopterDynamics {
    public:
        QuadcopterDynamics(Eigen::VectorXd x0, double dt);

        // Compute the forces for each motor
        void get_motor_forces(Eigen::VectorXd u);

        // Update the dynamics (discrete)
        void update_dynamics(Eigen::VectorXd u0);

        // Get the current quadcopter state
        Eigen::VectorXd get_state();

    private:

        // States
        Eigen::VectorXd x;
        Eigen::VectorXd x_motors;

        // Discrete time update
        double dt_;

        // Mass (kg)
        double m;

        // Gravity (m/s^2)
        double g;

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
};

#endif