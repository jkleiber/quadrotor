
#pragma once

#include <Arduino.h>

#include "control_utils.h"

#define MICROS_IN_A_SECOND (float)(1000.0 * 1000.0)

class PIDController
{
public:
    PIDController()
    {
        Kp_ = 0.0;
        Ki_ = 0.0;
        Kd_ = 0.0;
    }

    PIDController(const float &kp, const float &ki, const float &kd)
        : Kp_(kp), Ki_(ki), Kd_(kd)
    {
    }

    ~PIDController() {}

    void Reset();

    // PID computation types
    float PID(const float &setpoint, const float &x);
    float PIDRate(const float &setpoint, const float &x, const float &x_rate);

    // Settings
    void SetOutputLimits(const float &min, const float &max);
    void SetIntegratorLimits(const float &min, const float &max);
    void SetIntegratorLeak(const float &leak);

private:
    float Kp_;
    float Ki_;
    float Kd_;

    // Integral
    float integrator_ = 0.0;
    // Integrator settings
    float int_min_ = 1.0;
    float int_max_ = -1.0;
    // Leaky integrator (1 = no leak, 0 = no integrator).
    float leak_factor_ = 1.0;

    // Derivative
    float prev_error_ = 0.0;
    unsigned long prev_t_ = 0;

    // Output settings
    float output_min_ = 1.0;
    float output_max_ = -1.0;
};
