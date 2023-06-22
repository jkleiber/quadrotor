#include "pid.h"

void PIDController::Reset()
{
    prev_error_ = 0.0;
    integrator_ = 0.0;
}

float PIDController::PID(const float &setpoint, const float &x)
{
    // Compute error.
    float error = setpoint - x;

    // Proportional control
    float P = Kp_ * error;

    // Compute time difference
    unsigned long t = micros();
    float dt = (float)(t - prev_t_) / MICROS_IN_A_SECOND;
    prev_t_ = t;

    // Compute the integral.
    integrator_ += 0.5 * (error + prev_error_) * dt;

    // Apply integrator limits if needed
    if (int_min_ < int_max_)
    {
        Clamp(&integrator_, int_min_, int_max_);
    }

    // Integral control
    float I = Ki_ * integrator_;

    // Compute the derivative
    if (dt < 1e-6)
    {
        dt = 1e-6;
    }
    float slope = (error - prev_error_) / dt;

    // Derivative control
    float D = Kd_ * slope;

    // Compute the output.
    float output = P + I + D;

    // Assign the current error to be the next previous error.
    prev_error_ = error;

    // Apply limits if needed.
    if (output_min_ < output_max_)
    {
        Clamp(&output, output_min_, output_max_);
    }

    return output;
}

float PIDController::PIDRate(const float &setpoint, const float &x,
                             const float &x_rate)
{
    // Compute error.
    float error = setpoint - x;

    // Proportional control
    float P = Kp_ * error;

    // Compute time difference
    unsigned long t = micros();
    float dt = (float)(t - prev_t_) / MICROS_IN_A_SECOND;
    prev_t_ = t;

    // Apply leaky integral.
    integrator_ *= leak_factor_;

    // Compute the integral.
    integrator_ += 0.5 * (error + prev_error_) * dt;

    // Apply integrator limits if needed
    if (int_min_ < int_max_)
    {
        Clamp(&integrator_, int_min_, int_max_);
    }

    // Integral control
    float I = Ki_ * integrator_;

    // Derivative control using sensor rate.
    float D = Kd_ * x_rate;

    // Compute the output.
    float output = P + I + D;

    // Assign the current error to be the next previous error.
    prev_error_ = error;

    // Apply limits if needed.
    if (output_min_ < output_max_)
    {
        Clamp(&output, output_min_, output_max_);
    }

    return output;
}

void PIDController::SetOutputLimits(const float &min, const float &max)
{
    output_min_ = min;
    output_max_ = max;
}

void PIDController::SetIntegratorLimits(const float &min, const float &max)
{
    int_min_ = min;
    int_max_ = max;
}

void PIDController::SetIntegratorLeak(const float &leak)
{
    leak_factor_ = leak;
}
