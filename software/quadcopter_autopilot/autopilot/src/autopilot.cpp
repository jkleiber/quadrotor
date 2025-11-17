#include "autopilot/autopilot.h"

namespace quadcopter
{
void Autopilot::Init() { altitude_control_.Init(); }

void Autopilot::Run(const AutopilotInputs &inputs)
{
    AltitudeControlOutputs altitude_control_outputs{
        altitude_control_.ComputeControl(inputs.altitude_control_inputs)};

    ControlMixing(altitude_control_outputs);
}

void Autopilot::ControlMixing(
    const AltitudeControlOutputs &altitude_control_outputs)
{
    float throttle = altitude_control_outputs.throttle;
    float roll_output = 0.0;
    float pitch_output = 0.0;
    float yaw_output = 0.0;

    output_.front_left_motor =
        throttle - roll_output + pitch_output + yaw_output;
    output_.front_right_motor =
        throttle + roll_output + pitch_output - yaw_output;
    output_.back_left_motor =
        throttle - roll_output - pitch_output - yaw_output;
    output_.back_right_motor =
        throttle + roll_output - pitch_output + yaw_output;
}

} // namespace quadcopter
