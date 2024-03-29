#include "control_sim/control_sim_loop.h"

bool ControlSimLoop::InitSim()
{
    // Set time to 0.
    clk_->Update(0.0);

    // Set to not running
    is_running = false;

    // The simulation is in its initial state
    is_initial = true;

    // Previous print at -100 to print initial conditions
    prev_print_time = -100.0;

    // Initial Conditions
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
    x = x0;

    // Reset control
    // Control
    u = Eigen::VectorXd::Zero(4);

    // Initialize subsystems
    quadcopter.Init(x);
    ctrl.Init();

    // Initialize plotting
    xy_chart.Init("XY", "XY");
    height_chart.Init("Height", "Height");
    roll_chart.Init("Roll", "Roll");
    pitch_chart.Init("Pitch", "Pitch");
    yaw_chart.Init("Yaw", "Yaw");

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

        // Simulator is not in initial state anymore.
        is_initial = false;
    }
    else if (gui_events_->reset_sim)
    {
        // Resetting should stop the sim.
        is_running = false;
        is_initial = true;
        InitSim();
        ResetPlots();
    }

    // GUI for parameter tuning
    bool is_open = true;
    ImGui::Begin("Simulation Parameters", &is_open);
    ImGui::Spacing();
    if (ImGui::BeginTabBar("Parameters"))
    {
        // Allow control updates even if the sim is stopped.
        ctrl.UpdateParams(!is_running);

        // Update the parameters if the sim is in the initial state.
        quadcopter.UpdateParams(is_initial);

        // End the tab bar
        ImGui::EndTabBar();
    }

    // End parameter tuning GUI
    ImGui::End();

    // Run the simulator if it is on.
    if (is_running)
    {
        RunSimLoop();
    }
    else if (is_initial)
    {
        // If the simulator is in the initial condition, update the PID gains.
        ctrl.IdleLoop();
        quadcopter.IdleLoop();
    }

    // Show the plots
    View();

    return true;
}

bool ControlSimLoop::RunSimLoop()
{
    // Print periodically while the simulation runs
    if (clk_->GetTime() - prev_print_time > 1.0)
    {
        prev_print_time = clk_->GetTime();
        std::cout << "t=" << prev_print_time << "\troll=" << x(9)
                  << " pitch=" << x(10) << " yaw=" << x(11) << std::endl;
    }

    // Setpoints
    // Maintain level flight while taking off
    Eigen::VectorXd setpoints = Eigen::VectorXd::Zero(12);
    setpoints(8) = 1.0; // 1 meter height

    // Control loop
    u = ctrl.RunLoop(x, setpoints);

    // Quadcopter dynamics
    quadcopter.UpdateDynamics(u);

    // Get state variables
    x = quadcopter.GetState();

    // Increment time
    clk_->Increment();

    return true;
}

bool ControlSimLoop::View()
{
    // Update time
    float t = clk_->GetTime();

    bool rpy_plots_active = true;
    ImGui::Begin("RPY Plots", &rpy_plots_active);
    ImGui::Spacing();
    // Update the scrolling buffer for RPY states
    roll_chart.AddPoint(t, Utils::RadToDeg(x(9)));
    pitch_chart.AddPoint(t, Utils::RadToDeg(x(10)));
    yaw_chart.AddPoint(t, Utils::RadToDeg(x(11)));

    // Plot the scrolling charts
    roll_chart.Plot(t);
    pitch_chart.Plot(t);
    yaw_chart.Plot(t);
    ImGui::End();

    // Height and XY position
    bool xyz_plots_active = true;
    ImGui::Begin("XYZ Plots", &xyz_plots_active);
    ImGui::Spacing();
    // Update the scrolling buffer for XYZ states
    height_chart.AddPoint(t, x(8));
    xy_chart.AddPoint(x(6), x(7));

    // Plot the scrolling charts
    height_chart.Plot(t);
    xy_chart.Plot();
    ImGui::End();

    return true;
}

bool ControlSimLoop::ResetPlots()
{
    xy_chart.Reset();
    height_chart.Reset();
    roll_chart.Reset();
    pitch_chart.Reset();
    yaw_chart.Reset();

    return true;
}