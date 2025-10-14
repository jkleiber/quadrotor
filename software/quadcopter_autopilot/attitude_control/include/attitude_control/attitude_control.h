
#pragma once

#include <pid/pid.h>

#include "attitude_control/parameters.h"

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
    AttitudeControl(const float control_period)
        : roll_pid_(kleiber_control::PidParameters{.kp = kRollKp,
                                                   .ki = kRollKi,
                                                   .kd = kRollKd,
                                                   .kf = 0.0,
                                                   .min_out = -kMaxRollRateCmd,
                                                   .max_out = kMaxRollRateCmd},
                    control_period),
          pitch_pid_(
              kleiber_control::PidParameters{.kp = kPitchKp,
                                             .ki = kPitchKi,
                                             .kd = kPitchKd,
                                             .kf = 0.0,
                                             .min_out = -kMaxPitchRateCmd,
                                             .max_out = kMaxPitchRateCmd},
              control_period),
          yaw_pid_(kleiber_control::PidParameters{.kp = kYawKp,
                                                  .ki = kYawKi,
                                                  .kd = kYawKd,
                                                  .kf = 0.0,
                                                  .min_out = -kMaxYawRateCmd,
                                                  .max_out = kMaxYawRateCmd},
                   control_period),
          roll_rate_pid_(
              kleiber_control::PidParameters{.kp = kRollRateKp,
                                             .ki = kRollRateKi,
                                             .kd = kRollRateKd,
                                             .kf = kRollRateKf,
                                             .min_out = kRatePIDLower,
                                             .max_out = kRatePIDUpper},
              control_period),
          pitch_rate_pid_(
              kleiber_control::PidParameters{.kp = kPitchRateKp,
                                             .ki = kPitchRateKi,
                                             .kd = kPitchRateKd,
                                             .kf = kPitchRateKf,
                                             .min_out = kRatePIDLower,
                                             .max_out = kRatePIDUpper},
              control_period),
          yaw_rate_pid_(
              kleiber_control::PidParameters{.kp = kYawRateKp,
                                             .ki = kYawRateKi,
                                             .kd = kYawRateKd,
                                             .kf = kYawRateKf,
                                             .min_out = kRatePIDLower,
                                             .max_out = kRatePIDUpper},
              control_period),
          mode_(Rate)
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
    kleiber_control::Pid roll_pid_;
    kleiber_control::Pid pitch_pid_;
    kleiber_control::Pid yaw_pid_;
    kleiber_control::Pid roll_rate_pid_;
    kleiber_control::Pid pitch_rate_pid_;
    kleiber_control::Pid yaw_rate_pid_;

    // Control mode
    ControlMode mode_;
};