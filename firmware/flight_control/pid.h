
#pragma once

#include <Arduino.h>

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

    PIDController(float &kp, float &ki, float &kd) : Kp_(kp), Ki_(ki), Kd_(kd)
    {
    }

    ~PIDController() {}

    void Reset();
    void Clamp(float *val, float &min, float &max);

    // PID computation types
    float PID(float &setpoint, float &x);
    float PIDRate(float &setpoint, float &x, float &x_rate);

    // Settings
    void SetOutputLimits(float &min, float &max);
    void SetIntegratorLimits(float &min, float &max);

private:
    float Kp_;
    float Ki_;
    float Kd_;

    // Integral
    float integrator_ = 0.0;
    // Integrator settings
    float int_min_ = 1.0;
    float int_max_ = -1.0;

    // Derivative
    float prev_error_ = 0.0;
    unsigned long prev_t_ = 0;

    // Output settings
    float output_min_ = 1.0;
    float output_max_ = -1.0;
};
