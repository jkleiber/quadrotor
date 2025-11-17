#include "altitude_control/altitude_control.h"

#include <pid/parameters.h>

namespace quadcopter
{

void AltitudeControl::Init()
{
    altitude_pid_.Reset();
    kleiber_control::PidParameters params{.kp = 1.0,
                                          .ki = 0.0,
                                          .kd = 0.0,
                                          .kf = 0.0,
                                          .min_out = 0.0,
                                          .max_out = 1.0};
    altitude_pid_.SetParameters(params);
}

AltitudeControlOutputs
AltitudeControl::ComputeControl(const AltitudeControlInputs &inputs)
{
    AltitudeControlOutputs outputs{};
    outputs.throttle =
        altitude_pid_.UpdateOutput(inputs.height_setpoint, inputs.height);

    return outputs;
}

} // namespace quadcopter