#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

// TODO: move completely away from RobotLib implementation
#include "RobotLibUtil.h"
#include <chrono>
#include <algorithm>
#include <math.h>

#include "control_sim/sim_clock.h"


class PIDController
{
    public:
        // Constructors
        PIDController(SimClock *clk) : clk_(clk)
        {
            this->init();
            this->begin(0.0);
        }

        PIDController(SimClock *clk, float init_state, float kp, float ki, float kd) : clk_(clk)
        {
            this->init();
            this->begin(init_state, kp, ki, kd);
        }

        // Copy assignment
        void operator=(const PIDController& pid);

        // Init
        void init();

        // Setters
        void begin(float init_state);
        void begin(float init_state, float kp, float ki, float kd);
        void reset();
        void setBounded(bool bounded);
        void setOutputRange(float lower, float upper);
        void setTolerance(float setpoint_tol, float derivative_tol, bool apply_tolerance);
        void setIntegratorBounds(float min, float max);

        // Getters
        float update(float target_state, float cur_state);
        float getIntegratorValue();

    private:
        // PID values
        float derivative;
        float integrator;
        float error;

        // States
        float prev_error;
        float last_state;

        // Equation coefficients
        float kP;
        float kI;
        float kD;

        // Boundary control
        float high;
        float low;
        bool is_bounded;

        // Timing
        SimClock const* clk_;
        double last_time;
        float dt;

        // Tolerance control
        float setpoint_tolerance;
        float derivative_tolerance;
        bool tolerance_enabled;

        // Integrator controls
        float integrator_min;
        float integrator_max;
};

#endif // PIDCONTROLLER_H