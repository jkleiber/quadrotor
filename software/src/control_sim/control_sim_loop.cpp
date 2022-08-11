#include "control_sim/control_sim_loop.h"

bool ControlSimLoop::InitSim()
{
    // Set time to 0.
    clk_->update(0.0);

    // Set to not running
    is_running = false;

    // Previous print at -100 to print initial conditions
    prev_print_time = -100.0;

    // Initial Conditions
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
    x = x0;

    // Reset control
    // Control
    u = Eigen::VectorXd::Zero(4);

    // Initialize subsystems
    quadcopter.init(x);
    ctrl.init();

    return true;
}

bool ControlSimLoop::UpdateSim()
{
    // Process event signals from the GUI
    if (gui_events_->stop_sim)
    {
        is_running = false;
    }
    else if (gui_events_->start_sim)
    {
        is_running = true;
    }
    else if (gui_events_->reset_sim)
    {
        // Resetting should stop the sim.
        is_running = false;
        InitSim();
    }

    // Run the simulator if it is on.
    if (is_running)
    {
        RunSimLoop();
    }

    return true;
}

bool ControlSimLoop::RunSimLoop()
{
    // Print periodically while the simulation runs
    if (clk_->get_time() - prev_print_time > 1.0)
    {
        prev_print_time = clk_->get_time();
        std::cout << "t=" << prev_print_time << "\troll=" << x(9)
                  << " pitch=" << x(10) << " yaw=" << x(11) << std::endl;
    }

    // Setpoints
    // Maintain level flight while taking off
    Eigen::VectorXd setpoints = Eigen::VectorXd::Zero(12);
    setpoints(8) = 1.0; // 1 meter height

    // Control loop
    u = ctrl.run_loop(x, setpoints);

    // Quadcopter dynamics
    quadcopter.update_dynamics(u);

    // Get state variables
    x = quadcopter.get_state();

    // Update time
    float t = clk_->get_time();

    // Update the scrolling buffer for RPY states
    roll_chart.AddPoint(t, Utils::RadToDeg(x(9)));
    pitch_chart.AddPoint(t, Utils::RadToDeg(x(10)));
    yaw_chart.AddPoint(t, Utils::RadToDeg(x(11)));

    // Plot the scrolling charts
    roll_chart.Plot(t);
    pitch_chart.Plot(t);
    yaw_chart.Plot(t);

    // Increment time
    clk_->increment();

    return true;
}