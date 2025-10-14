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

    // Initialize logging.
    InitLogging();

    return true;
}

void ControlSimLoop::InitLogging()
{
    // Initialize logging
    bool is_log_active = false;
    state_log_.LoggingActive(&is_log_active);
    if (is_log_active)
    {
        state_log_.CloseLog();
    }
    // TODO: time based logging
    state_log_.Init("state_log_.csv");
    state_log_.LogHeaders("u, v, w, p, q, r, x, y, z, roll, pitch, yaw");
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
        UpdateQuadcopterParams(is_initial);

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

    // Log the current sim state.
    LogState();

    // Increment time
    clk_->Increment();

    return true;
}

void ControlSimLoop::LogState()
{
    // Log the current state.
    state_log_.LogVectorXd(this->x);
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

void ControlSimLoop::UpdateQuadcopterParams(bool is_enabled)
{
    quadcopter::VehicleParameters quadcopter_params{
        quadcopter.GetVehicleParameters()};
    quadcopter::EnvironmentParameters env_params{
        quadcopter.GetEnvironmentParameters()};

    // If the simulation is running, disable gain selection
    if (!is_enabled)
    {
        ImGui::BeginDisabled();
    }

    // Add a tab to the TabBar
    // Note: there must be a BeginTabBar before this is called.
    if (ImGui::BeginTabItem("Vehicle"))
    {
        if (ImGui::CollapsingHeader("Mass and Inertials"))
        {
            ImGui::InputScalar("Mass (kg)", ImGuiDataType_Double,
                               &quadcopter_params.m, NULL);
            ImGui::InputScalar("Ixx (kg/m^2)", ImGuiDataType_Double,
                               &quadcopter_params.Ixx, NULL);
            ImGui::InputScalar("Iyy (kg/m^2)", ImGuiDataType_Double,
                               &quadcopter_params.Iyy, NULL);
            ImGui::InputScalar("Izz (kg/m^2)", ImGuiDataType_Double,
                               &quadcopter_params.Izz, NULL);
        }
        if (ImGui::CollapsingHeader("Dimensions"))
        {
            ImGui::InputScalar("Arm dx (m)", ImGuiDataType_Double,
                               &quadcopter_params.dx_arm, NULL);
            ImGui::InputScalar("Arm dy (m)", ImGuiDataType_Double,
                               &quadcopter_params.dy_arm, NULL);
        }
        if (ImGui::CollapsingHeader("Limits"))
        {
            ImGui::InputScalar("Max Motor Force (N)", ImGuiDataType_Double,
                               &quadcopter_params.max_motor_force, NULL);
            ImGui::InputScalar("Max Ang. Vel (rad/s)", ImGuiDataType_Double,
                               &quadcopter_params.max_omega, NULL);
            ImGui::InputScalar("Max Slew Rate (%/s)", ImGuiDataType_Double,
                               &quadcopter_params.motor_slew_rate, NULL);
        }
        if (ImGui::CollapsingHeader("Dynamic Constants"))
        {
            ImGui::InputScalar("Kf divisor", ImGuiDataType_Double,
                               &quadcopter_params.Kf_divisor_, NULL);
        }
        if (ImGui::CollapsingHeader("Environment"))
        {
            ImGui::InputScalar("Gravity (m/s^2)", ImGuiDataType_Double,
                               &env_params.g, NULL);
            ImGui::InputScalar("Disturbance Mean", ImGuiDataType_Double,
                               &env_params.dist_mean, NULL);
            ImGui::InputScalar("Disturbance Std. Dev", ImGuiDataType_Double,
                               &env_params.dist_stddev, NULL);
        }
        ImGui::EndTabItem();
    }

    // End the disabled part if gains should not be changed
    if (!is_enabled)
    {
        ImGui::EndDisabled();
    }
    else
    {
        quadcopter.SetVehicleParams(quadcopter_params);
        quadcopter.SetEnvironmentParams(env_params);
    }
}
