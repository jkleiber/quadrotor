#ifndef QUADCOPTER_AUTOPILOT_AUTOPILOT_H
#define QUADCOPTER_AUTOPILOT_AUTOPILOT_H

#include <altitude_control/altitude_control.h>

namespace quadcopter
{

typedef struct autopilot_inputs_t
{
    AltitudeControlInputs altitude_control_inputs;
} AutopilotInputs;

typedef struct autopilot_state_t
{
    float throttle_for_lift = 0.0;
} AutopilotState;

typedef struct autopilot_outputs_t
{
    float front_left_motor = 0.0;
    float front_right_motor = 0.0;
    float back_left_motor = 0.0;
    float back_right_motor = 0.0;
} AutopilotOutputs;

class Autopilot
{
public:
    explicit Autopilot(AutopilotOutputs &output) : output_(output) { Init(); }

    void Init();
    void Run(const AutopilotInputs &input);

private:
    // Interface + local state.
    AutopilotOutputs &output_;
    AutopilotState state_;

    // Controllers.
    AltitudeControl altitude_control_;

    /**
     *
     */
    void ControlMixing(const AltitudeControlOutputs &altitude_control_outputs);
};
} // namespace quadcopter

#endif