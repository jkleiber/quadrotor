
#include <chrono>
#include <iostream>
#include <thread>

#include <autopilot/autopilot.h>
#include <quadcopter_simulator/quadcopter_simulator.h>

#include <Eigen/Dense>

static quadcopter::AutopilotInputs autopilot_inputs_{};
static quadcopter::AutopilotOutputs autopilot_outputs_{};

void RunAutopilot(const QuadcopterSim &sim, quadcopter::Autopilot &autopilot)
{
    // Get state directly from the sim rather than having sensors.
    // TODO: simulate sensors.
    const SimState sim_state{sim.GetSimState()};

    autopilot_inputs_.altitude_control_inputs.height = sim_state.altitude;
    autopilot_inputs_.altitude_control_inputs.height_setpoint = 1.0;
    autopilot.Run(autopilot_inputs_);
}

int main(int argc, char **argv)
{
    // Timing
    double dt = 0.01;

    // Simulation clock
    SimClock clk(dt);

    // Control simulation loop
    QuadcopterSim sim(clk);

    // Autopilot.
    quadcopter::Autopilot autopilot{autopilot_outputs_};

    bool is_running = true;
    while (is_running)
    {
        RunAutopilot(sim, autopilot);

        // Update the simulation.
        sim.Update();
        sim.ApplyMotorControl(
            static_cast<double>(autopilot_outputs_.front_left_motor),
            static_cast<double>(autopilot_outputs_.front_right_motor),
            static_cast<double>(autopilot_outputs_.back_left_motor),
            static_cast<double>(autopilot_outputs_.back_right_motor));
    }

    return 0;
}
