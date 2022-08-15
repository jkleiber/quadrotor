#include "control_sim/control_loop.h"

void ControlLoop::Init()
{
    // Choose the default gains if we haven't enabled them yet.
    // This only runs once so an initial gainset exists.
    if (is_default)
    {
        SetDefaultGains();
        is_default = false;
    }
    // Otherwise, whatever stored gains from the GUI will be used.

    // Apply the gains to the PID
    ApplyGains();

    // Set up logging
    bool is_log_active = false;
    ctrl_log.LoggingActive(&is_log_active);
    if (is_log_active)
    {
        ctrl_log.CloseLog();
    }

    // TODO: time based logging
    ctrl_log.Init("ctrl_log.csv");

    // Set up csv logging headers
    ctrl_log.LogHeaders("motor_fr, motor_bl, motor_fl, motor_br");
}

void ControlLoop::IdleLoop()
{
    // Constantly update the PID gains while the simulation is not running
    ApplyGains();
}

Eigen::VectorXd ControlLoop::RunLoop(Eigen::VectorXd x,
                                      Eigen::VectorXd setpoints)
{
    // Setpoints correspond to states directly
    Eigen::VectorXd error = setpoints - x;

    // Height / Throttle PID
    double height = x(8);
    double height_target = setpoints(8);
    double throttle = height_pid_ff + height_pid.Update(height_target, height);
    throttle = std::min(1.0, std::max(0.0, throttle)); // Ensure between 0 and 1

    // Pitch PID
    double pitch = x(10);
    double pitch_target = setpoints(10);
    double pitch_output = pitch_pid.Update(pitch_target, pitch);

    // Roll PID
    double roll = x(9);
    double roll_target = setpoints(9);
    double roll_output = roll_pid.Update(roll_target, roll);

    // Yaw PID
    double yaw = x(11);
    double yaw_target = setpoints(11);
    double yaw_output = yaw_pid.Update(yaw_target, yaw);

    // Motor throttle vector
    Eigen::VectorXd motor_pows = Eigen::VectorXd::Zero(4);

    // Throttle allocation for each motor
    motor_pows(2) =
        throttle - roll_output + pitch_output + yaw_output; // front left
    motor_pows(1) =
        throttle - roll_output - pitch_output - yaw_output; // back left
    motor_pows(0) =
        throttle + roll_output + pitch_output - yaw_output; // front right
    motor_pows(3) =
        throttle + roll_output - pitch_output + yaw_output; // back right

    // Ensure each motor has non-negative output
    for (int i = 0; i < 4; ++i)
    {
        motor_pows(i) = std::min(1.0, std::max(0.0, motor_pows(i)));
    }

    // Log control
    ctrl_log.LogVectorXd(motor_pows);

    // Set throttle
    return motor_pows;
}

bool ControlLoop::UpdateParams(bool enabled)
{
    // If the simulation is running, disable gain selection
    if (!enabled)
        ImGui::BeginDisabled();

    // Add a tab to the TabBar
    // Note: there must be a BeginTabBar before this is called.
    if (ImGui::BeginTabItem("Controllers"))
    {
        if (ImGui::CollapsingHeader("Roll PID"))
        {
            ImGui::InputScalar("Roll Kp", ImGuiDataType_Double, &roll_gains.kp, NULL);
            ImGui::InputScalar("Roll Ki", ImGuiDataType_Double, &roll_gains.ki, NULL);
            ImGui::InputScalar("Roll Kd", ImGuiDataType_Double, &roll_gains.kd, NULL);
            ImGui::InputScalar("Roll Min Output", ImGuiDataType_Double, &roll_gains.min_out, NULL);
            ImGui::InputScalar("Roll Max Output", ImGuiDataType_Double, &roll_gains.max_out, NULL);
        }
        if (ImGui::CollapsingHeader("Pitch PID"))
        {
            ImGui::InputScalar("Pitch Kp", ImGuiDataType_Double, &pitch_gains.kp, NULL);
            ImGui::InputScalar("Pitch Ki", ImGuiDataType_Double, &pitch_gains.ki, NULL);
            ImGui::InputScalar("Pitch Kd", ImGuiDataType_Double, &pitch_gains.kd, NULL);
            ImGui::InputScalar("Pitch Min Output", ImGuiDataType_Double, &pitch_gains.min_out, NULL);
            ImGui::InputScalar("Pitch Max Output", ImGuiDataType_Double, &pitch_gains.max_out, NULL);
        }
        if (ImGui::CollapsingHeader("Yaw PID"))
        {
            ImGui::InputScalar("Yaw Kp", ImGuiDataType_Double, &yaw_gains.kp, NULL);
            ImGui::InputScalar("Yaw Ki", ImGuiDataType_Double, &yaw_gains.ki, NULL);
            ImGui::InputScalar("Yaw Kd", ImGuiDataType_Double, &yaw_gains.kd, NULL);
            ImGui::InputScalar("Yaw Min Output", ImGuiDataType_Double, &yaw_gains.min_out, NULL);
            ImGui::InputScalar("Yaw Max Output", ImGuiDataType_Double, &yaw_gains.max_out, NULL);
        }
        if (ImGui::CollapsingHeader("Height PID"))
        {
            ImGui::InputScalar("Height Kp", ImGuiDataType_Double, &height_gains.kp, NULL);
            ImGui::InputScalar("Height Ki", ImGuiDataType_Double, &height_gains.ki, NULL);
            ImGui::InputScalar("Height Kd", ImGuiDataType_Double, &height_gains.kd, NULL);
            ImGui::InputScalar("Height Kf", ImGuiDataType_Double, &height_gains.kf, NULL);
            ImGui::InputScalar("Height Min Output", ImGuiDataType_Double, &height_gains.min_out, NULL);
            ImGui::InputScalar("Height Max Output", ImGuiDataType_Double, &height_gains.max_out, NULL);
        }
        ImGui::EndTabItem();
    }

    // End the disabled part if gains should not be changed
    if (!enabled)
        ImGui::EndDisabled();

    return true;
}

void ControlLoop::SetDefaultGains()
{
    // Roll PID
    roll_gains.kp = 0.1;
    roll_gains.ki = 0.001;
    roll_gains.kd = 0.05;
    roll_gains.kf = 0.0;
    roll_gains.min_out = -0.2;
    roll_gains.max_out = 0.2;

    // Pitch PID
    pitch_gains.kp = 0.1;
    pitch_gains.ki = 0.001;
    pitch_gains.kd = 0.05;
    pitch_gains.kf = 0.0;
    pitch_gains.min_out = -0.2;
    pitch_gains.max_out = 0.2;

    // Yaw PID
    yaw_gains.kp = 0.1;
    yaw_gains.ki = 0.001;
    yaw_gains.kd = 0.05;
    yaw_gains.kf = 0.0;
    yaw_gains.min_out = -0.2;
    yaw_gains.max_out = 0.2;

    // Height PID
    height_gains.kp = 0.1;
    height_gains.ki = 0.001;
    height_gains.kd = 0.1;
    height_gains.kf = 0.53;
    height_gains.min_out = -0.1;
    height_gains.max_out = 0.1;
    // height_pid.Start(0, 0.175, 0.001, 0.05); // Noiseless Height PID
}

void ControlLoop::ApplyGains()
{
    // Apply whatever gains are currently loaded.

    // Init Height PID
    // NOTE: Height PID simulates pilot flying ability for now
    height_pid.Start(0, height_gains.kp, height_gains.ki, height_gains.kd);
    height_pid.SetOutputRange(height_gains.min_out, height_gains.max_out);

    // Set feedforward to maintain constant height
    height_pid_ff = height_gains.kf;

    // Init roll PID
    roll_pid.Start(0, roll_gains.kp, roll_gains.ki, roll_gains.kd);
    roll_pid.SetOutputRange(roll_gains.min_out, roll_gains.max_out);

    // Init pitch PID
    pitch_pid.Start(0, pitch_gains.kp, pitch_gains.ki, pitch_gains.kd);
    pitch_pid.SetOutputRange(pitch_gains.min_out, pitch_gains.max_out);

    // Init yaw PID
    yaw_pid.Start(0, yaw_gains.kp, yaw_gains.ki, yaw_gains.kd);
    yaw_pid.SetOutputRange(yaw_gains.min_out, yaw_gains.max_out);

    // Save the gains
    SaveGains();
}

void ControlLoop::SaveGains()
{
    std::string gain_string = ""
    "// Roll PID\n"
    "roll_gains.kp = " + std::to_string(roll_gains.kp) + ";\n"
    "roll_gains.ki = " + std::to_string(roll_gains.ki) + ";\n"
    "roll_gains.kd = " + std::to_string(roll_gains.kd) + ";\n"
    "roll_gains.kf = " + std::to_string(roll_gains.kf) + ";\n"
    "roll_gains.min_out = " + std::to_string(roll_gains.min_out) + ";\n"
    "roll_gains.max_out = " + std::to_string(roll_gains.max_out) + ";\n"
    "\n"
    "// Pitch PID\n"
    "pitch_gains.kp = " + std::to_string(pitch_gains.kp) + ";\n"
    "pitch_gains.ki = " + std::to_string(pitch_gains.ki) + ";\n"
    "pitch_gains.kd = " + std::to_string(pitch_gains.kd) + ";\n"
    "pitch_gains.kf = " + std::to_string(pitch_gains.kf) + ";\n"
    "pitch_gains.min_out = " + std::to_string(pitch_gains.min_out) + ";\n"
    "pitch_gains.max_out = " + std::to_string(pitch_gains.max_out) + ";\n"
    "\n"
    "// Yaw PID\n"
    "yaw_gains.kp = " + std::to_string(yaw_gains.kp) + ";\n"
    "yaw_gains.ki = " + std::to_string(yaw_gains.ki) + ";\n"
    "yaw_gains.kd = " + std::to_string(yaw_gains.kd) + ";\n"
    "yaw_gains.kf = " + std::to_string(yaw_gains.kf) + ";\n"
    "yaw_gains.min_out = " + std::to_string(yaw_gains.min_out) + ";\n"
    "yaw_gains.max_out = " + std::to_string(yaw_gains.max_out) + ";\n"
    "\n"
    "// Height PID\n"
    "height_gains.kp = " + std::to_string(height_gains.kp) + ";\n"
    "height_gains.ki = " + std::to_string(height_gains.ki) + ";\n"
    "height_gains.kd = " + std::to_string(height_gains.kd) + ";\n"
    "height_gains.kf = " + std::to_string(height_gains.kf) + ";\n"
    "height_gains.min_out = " + std::to_string(height_gains.min_out) + ";\n"
    "height_gains.max_out = " + std::to_string(height_gains.max_out) + ";\n";

    // Save this string to the gains file (to load as default via copy-paste later.)
    // Gains export to TOML.
    // TODO: import from TOML
    gain_file.open("gains.toml", std::ofstream::out | std::ofstream::trunc);
    gain_file << gain_string << std::flush;
    gain_file.close();
}
