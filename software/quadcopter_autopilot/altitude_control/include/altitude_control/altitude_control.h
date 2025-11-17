#ifndef QUADCOPTER_AUTOPILOT_ALTITUDE_CONTROL_H
#define QUADCOPTER_AUTOPILOT_ALTITUDE_CONTROL_H

#include <pid/pid.h>

namespace quadcopter
{
typedef struct altitude_control_inputs_t
{
    // States.
    float height;

    // Setpoints.
    float height_setpoint;

} AltitudeControlInputs;

typedef struct altitude_control_outputs_t
{
    float throttle;
} AltitudeControlOutputs;

class AltitudeControl
{
public:
    AltitudeControl() {}

    void Init();
    AltitudeControlOutputs ComputeControl(const AltitudeControlInputs &inputs);

private:
    kleiber_control::Pid altitude_pid_;
};
} // namespace quadcopter

#endif