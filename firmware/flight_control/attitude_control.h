
#pragma once

#include "constants.h"
#include "pid.h"

typedef struct attitude_control_inputs_t
{
    // angular rates
    float p;
    float q;
    float r;

    // angles
    float roll;
    float pitch;
    float yaw;

    // remote control stick values
    float roll_stick;
    float pitch_stick;
    float yaw_stick;
} AttitudeControlInputs;

typedef struct attitude_control_outputs_t
{
    // Motor outputs
    float roll_output;
    float pitch_output;
    float yaw_output;

    // Control setpoints
    float p_setpoint;
    float q_setpoint;
    float r_setpoint;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
} AttitudeControlOutputs;

class AttitudeControl
{
public:
    AttitudeControl()
        : roll_pid_(kRollKp, kRollKi, kRollKd),
          pitch_pid_(kPitchKp, kPitchKi, kPitchKd),
          yaw_pid_(kYawKp, kYawKi, kYawKd),
          roll_rate_pid_(kRollRateKp, kRollRateKi, kRollRateKd),
          pitch_rate_pid_(kPitchRateKp, kPitchRateKi, kPitchRateKd),
          yaw_rate_pid_(kYawRateKp, kYawRateKi, kYawRateKd), mode_(Rate)
    {
    }

    ~AttitudeControl() {}

    void Init();
    void Reset();
    void SetMode(const ControlMode &mode);

    void Process(const AttitudeControlInputs &inputs,
                 AttitudeControlOutputs *outputs);

private:
    // Helper function to convert [0, 1] ranges into [x, y] ranges.
    float ConvertPercentToRange(const float &pct, const float &min,
                                const float &max);

    // PID Controllers
    PIDController roll_pid_;
    PIDController pitch_pid_;
    PIDController yaw_pid_;
    PIDController roll_rate_pid_;
    PIDController pitch_rate_pid_;
    PIDController yaw_rate_pid_;

    // Control mode
    ControlMode mode_;
};