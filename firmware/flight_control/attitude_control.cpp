
#include "attitude_control.h"

void AttitudeControl::Init()
{
    // Set PID output limits
    roll_pid_.SetOutputLimits(-kMaxRollRateCmd, kMaxRollRateCmd);
    pitch_pid_.SetOutputLimits(-kMaxPitchRateCmd, kMaxPitchRateCmd);
    yaw_pid_.SetOutputLimits(-kMaxYawRateCmd, kMaxYawRateCmd);
    roll_rate_pid_.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    pitch_rate_pid_.SetOutputLimits(kRatePIDLower, kRatePIDUpper);
    yaw_rate_pid_.SetOutputLimits(kRatePIDLower, kRatePIDUpper);

    // Set the PID integrator limits
    roll_rate_pid_.SetIntegratorLimits(-kRollRateIntegralLimit,
                                       kRollRateIntegralLimit);
    pitch_rate_pid_.SetIntegratorLimits(-kPitchRateIntegralLimit,
                                        kPitchRateIntegralLimit);
    yaw_rate_pid_.SetIntegratorLimits(-kYawRateIntegralLimit,
                                      kYawRateIntegralLimit);

    // Set the leakiness for the integrators.
    roll_pid_.SetIntegratorLeak(kRollLeak);
    pitch_pid_.SetIntegratorLeak(kPitchLeak);
    yaw_pid_.SetIntegratorLeak(kYawLeak);
    roll_rate_pid_.SetIntegratorLeak(kRollRateLeak);
    pitch_rate_pid_.SetIntegratorLeak(kPitchRateLeak);
    yaw_rate_pid_.SetIntegratorLeak(kYawRateLeak);
}

void AttitudeControl::SetMode(const ControlMode &mode)
{
    // Set the control mode.
    mode_ = mode;
}

void AttitudeControl::Process(const AttitudeControlInputs &inputs,
                              AttitudeControlOutputs *outputs)
{
    // Don't do anything if the outputs pointer is nullptr.
    if (outputs == nullptr)
    {
        return;
    }

    // OUTER LOOP
    switch (kMode)
    {
    case Angle:
        outputs->roll_setpoint =
            ConvertPercentToRange(inputs.roll_stick, -kMaxRollCmd, kMaxRollCmd);
        outputs->pitch_setpoint = ConvertPercentToRange(
            inputs.pitch_stick, -kMaxPitchCmd, kMaxPitchCmd);

        // Outer loop PID controllers
        outputs->p_setpoint =
            roll_pid_.PIDRate(outputs->roll_setpoint, inputs.roll, inputs.p);
        outputs->q_setpoint =
            pitch_pid_.PIDRate(outputs->pitch_setpoint, inputs.pitch, inputs.q);
        outputs->r_setpoint = 0;

        break;

    case Rate:
    default:
        outputs->p_setpoint = ConvertPercentToRange(
            inputs.roll_stick, -kMaxRollRateCmd, kMaxRollRateCmd);
        outputs->q_setpoint = ConvertPercentToRange(
            inputs.pitch_stick, -kMaxPitchRateCmd, kMaxPitchRateCmd);
        outputs->r_setpoint = 0;

        // Apply deadbands.
        if (fabs(outputs->p_setpoint) < kRateDeadband)
        {
            outputs->p_setpoint = 0.0;
        }
        if (fabs(outputs->q_setpoint) < kRateDeadband)
        {
            outputs->q_setpoint = 0.0;
        }
    }

    // INNER LOOP
    outputs->roll_output = roll_rate_pid_.PID(outputs->p_setpoint, inputs.p) +
                           (outputs->p_setpoint * kRollRateKf);
    outputs->pitch_output = pitch_rate_pid_.PID(outputs->q_setpoint, inputs.q) +
                            (outputs->q_setpoint * kPitchRateKf);
    outputs->yaw_output = yaw_rate_pid_.PID(outputs->r_setpoint, inputs.r) +
                          (outputs->r_setpoint * kYawRateKf);
    // Note: yaw_output is from the yaw rate PID, which tries to maintain 0 yaw
    // rate. Eventually an outer loop yaw controller will be added to command
    // yaw rate based on yaw setpoint.

    // TEST OVERRIDES
    outputs->pitch_output = 0.0;
    outputs->yaw_output = 0.0;
}

void AttitudeControl::Reset()
{
    roll_pid_.Reset();
    pitch_pid_.Reset();
    yaw_pid_.Reset();

    roll_rate_pid_.Reset();
    pitch_rate_pid_.Reset();
    yaw_rate_pid_.Reset();
}

float AttitudeControl::ConvertPercentToRange(const float &pct, const float &min,
                                             const float &max)
{
    float range = max - min;
    float value = min + (pct * range);

    return value;
}