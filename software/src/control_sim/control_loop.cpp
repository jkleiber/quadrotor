#include "control_sim/control_loop.h"

void ControlLoop::init()
{
    // Init Height PID
    // NOTE: Height PID simulates pilot flying ability for now
    // height_pid.begin(0, 0.175, 0.001, 0.05); // Noiseless Height PID
    height_pid.begin(0, 0.1, 0.001, 0.1);
    height_pid.setOutputRange(-0.1, 0.1);

    // Set feedforward to maintain constant height
    height_pid_ff = 0.53;

    // Init pitch PID
    pitch_pid.begin(0, 0.1, 0.001, 0.05);
    pitch_pid.setOutputRange(-0.2, 0.2);

    // Init roll PID
    roll_pid.begin(0, 0.1, 0.001, 0.05);
    roll_pid.setOutputRange(-0.2, 0.2);

    // Init yaw PID
    yaw_pid.begin(0, 0.1, 0.001, 0.05);
    yaw_pid.setOutputRange(-0.2, 0.2);

    // Set up logging
    bool is_log_active = false;
    ctrl_log.logging_active(&is_log_active);
    if (is_log_active)
    {
        ctrl_log.close_log();
    }

    // TODO: time based logging
    ctrl_log.init("ctrl_log.csv");

    // Set up csv logging headers
    ctrl_log.log_headers("motor_fr, motor_bl, motor_fl, motor_br");
}

Eigen::VectorXd ControlLoop::run_loop(Eigen::VectorXd x,
                                      Eigen::VectorXd setpoints)
{
    // Setpoints correspond to states directly
    Eigen::VectorXd error = setpoints - x;

    // Height / Throttle PID
    double height = x(8);
    double height_target = setpoints(8);
    double throttle = height_pid_ff + height_pid.update(height_target, height);
    throttle = std::min(1.0, std::max(0.0, throttle)); // Ensure between 0 and 1

    // Pitch PID
    double pitch = x(10);
    double pitch_target = setpoints(10);
    double pitch_output = pitch_pid.update(pitch_target, pitch);

    // Roll PID
    double roll = x(9);
    double roll_target = setpoints(9);
    double roll_output = roll_pid.update(roll_target, roll);

    // Yaw PID
    double yaw = x(11);
    double yaw_target = setpoints(11);
    double yaw_output = yaw_pid.update(yaw_target, yaw);

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
    ctrl_log.log_vector_xd(motor_pows);

    // Set throttle
    return motor_pows;
}

bool ControlLoop::UpdateParams()
{
    bool is_open = false;
    ImGui::Begin("Control Parameters", &is_open);
    ImGui::Spacing();
    if (ImGui::BeginTabBar("Control Tuning"))
    {
        if (ImGui::BeginTabItem("Controllers"))
        {
            if (ImGui::CollapsingHeader("Roll PID"))
            {
            }
            if (ImGui::CollapsingHeader("Pitch PID"))
            {
            }
            if (ImGui::CollapsingHeader("Yaw PID"))
            {
            }
            if (ImGui::CollapsingHeader("Height PID"))
            {
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();

    return true;
}