#include "control_sim/quadcopter_dynamics.h"

void QuadcopterDynamics::Init(Eigen::VectorXd x0)
{
    // Initial conditions
    this->x = x0; // Expecting a R^12 vector

    this->dt_ = clk_->GetDt();

    // On the first run, load the defaults for now
    if (is_default)
    {
        LoadDefaultParams();
        is_default = false;
    }

    // Load the vehicle parameters
    SetVehicleParams();

    // Motors initial conditions (assumed to be 0)
    this->x_motors = Eigen::VectorXd::Zero(4);
    this->u_prev = Eigen::VectorXd::Zero(4);

    // Initialize logging
    bool is_log_active = false;
    state_log.LoggingActive(&is_log_active);
    if (is_log_active)
    {
        state_log.CloseLog();
    }
    // TODO: time based logging
    state_log.Init("state_log.csv");
    state_log.LogHeaders("u, v, w, p, q, r, x, y, z, roll, pitch, yaw");
}

void QuadcopterDynamics::GetMotorForces(Eigen::VectorXd u)
{
    // TODO: add a delay into the motor commands

    // Find the slew rate limits
    Eigen::VectorXd u_max_slew = motor_slew_rate * Eigen::VectorXd::Ones(4);
    Eigen::VectorXd u_max = u_prev + u_max_slew;
    Eigen::VectorXd u_min = u_prev - u_max_slew;

    // Slew rate limit the motors
    for (int i = 0; i < 4; ++i)
    {
        u(i) = std::min(u_max(i), std::max(u_min(i), u(i)));
    }

    // u = throttle for each motor
    // Convert throttle to rad/s
    Eigen::VectorXd w_motors = max_omega * u;

    // Thrust = K_f * w_i^2 for the i-th motor
    Eigen::VectorXd new_x_motors = K_f * w_motors.array().pow(2);

    // Disturb thrust generated
    for (int i = 0; i < 4; ++i)
    {
        std::normal_distribution<double> motor_dist(dist_mean, dist_stddev);
        double dist = motor_dist(motor_gen);

        // Disturb
        new_x_motors(i) += dist;
    }

    this->x_motors = new_x_motors;
    u_prev = u;
}

void QuadcopterDynamics::UpdateDynamics(Eigen::VectorXd u0)
{
    // States
    double u = x(0);
    double v = x(1);
    double w = x(2);
    double p = x(3);
    double q = x(4);
    double r = x(5);
    double x_nav = x(6);
    double y_nav = x(7);
    double z_nav = x(8);
    double roll = x(9);
    double pitch = x(10);
    double yaw = x(11);

    // Compute motor forces from control.
    this->GetMotorForces(u0);

    // Motor Numbering
    /*
        3 --[---]---1
            |   |
        2---[---]---4
    */
    // Roll to the right is positive
    // Pitch up is positive
    // CW yaw is positive

    // Control is the motor forces
    double F1 = x_motors(0);
    double F2 = x_motors(1);
    double F3 = x_motors(2);
    double F4 = x_motors(3);

    double arm_cg_len = sqrt(pow(dx_arm, 2) + pow(dy_arm, 2));

    // "Virtual" control
    double Fz = F1 + F2 + F3 + F4;
    double L = (F1 - F2 - F3 + F4) * dy_arm;
    double M = (F1 - F2 + F3 - F4) * dx_arm;

    // w^2 = F / K_f
    // torque = K_m w^2 = K_b (F / K_f)
    // Note: this assumes CW is positive
    double N = K_m * (-F1 - F2 + F3 + F4) / K_f;

    // Angles
    double cos_roll = cos(roll);
    double cos_pitch = cos(pitch);
    double cos_yaw = cos(yaw);
    double sin_roll = sin(roll);
    double sin_pitch = sin(pitch);
    double sin_yaw = sin(yaw);

    // Derivative state
    Eigen::VectorXd x_dot = Eigen::VectorXd::Zero(12);

    // Velocity update
    x_dot(0) = -g * sin_pitch + r * v - q * w;
    x_dot(1) = g * sin_roll * cos_pitch - r * u + p * w;
    x_dot(2) = (1 / m) * Fz - g * cos_roll * cos_pitch - q * u + p * v;

    // Angular velocity update
    x_dot(3) = (1 / Ixx) * (L + (Iyy - Izz) * q * r);
    x_dot(4) = (1 / Iyy) * (M + (Izz - Ixx) * p * r);
    x_dot(5) = (1 / Izz) * (N + (Ixx - Iyy) * p * q);

    // Position update
    x_dot(6) = cos_pitch * cos_yaw * u +
               (-cos_roll * sin_yaw + sin_roll * sin_pitch * cos_yaw) * v +
               (sin_roll * sin_yaw + cos_roll * sin_pitch * cos_yaw) * w;
    x_dot(7) = cos_pitch * sin_yaw * u +
               (cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw) * v +
               (-sin_roll * cos_yaw + cos_roll * sin_pitch * sin_yaw) * w;
    x_dot(8) =
        -sin_pitch * u + sin_roll * cos_pitch * v + cos_roll * cos_pitch * w;

    // Angle update
    x_dot(9) = p + (q * sin_roll + r * cos_roll) * tan(pitch);
    x_dot(10) = q * cos_pitch - r * sin_roll;
    x_dot(11) = (q * sin_roll + r * cos_roll) * (1 / cos_pitch);

    // Apply the update to the dynamics
    this->x = this->x + x_dot * dt_;

    // Log the dynamics
    state_log.LogVectorXd(this->x);

    // Don't allow the drone to fall below the ground
    // if (x(8) < 0)
    // {
    //     x(8) = 0;
    // }
}

bool QuadcopterDynamics::UpdateParams(bool is_enabled)
{
    // If the simulation is running, disable gain selection
    if (!is_enabled)
        ImGui::BeginDisabled();

    // Add a tab to the TabBar
    // Note: there must be a BeginTabBar before this is called.
    if (ImGui::BeginTabItem("Vehicle"))
    {
        if (ImGui::CollapsingHeader("Mass and Inertials"))
        {
            ImGui::InputScalar("Mass (kg)", ImGuiDataType_Double, &m, NULL);
            ImGui::InputScalar("Ixx (kg/m^2)", ImGuiDataType_Double, &Ixx,
                               NULL);
            ImGui::InputScalar("Iyy (kg/m^2)", ImGuiDataType_Double, &Iyy,
                               NULL);
            ImGui::InputScalar("Izz (kg/m^2)", ImGuiDataType_Double, &Izz,
                               NULL);
        }
        if (ImGui::CollapsingHeader("Dimensions"))
        {
            ImGui::InputScalar("Arm dx (m)", ImGuiDataType_Double, &dx_arm,
                               NULL);
            ImGui::InputScalar("Arm dy (m)", ImGuiDataType_Double, &dy_arm,
                               NULL);
        }
        if (ImGui::CollapsingHeader("Limits"))
        {
            ImGui::InputScalar("Max Motor Force (N)", ImGuiDataType_Double,
                               &max_motor_force, NULL);
            ImGui::InputScalar("Max Ang. Vel (rad/s)", ImGuiDataType_Double,
                               &max_omega, NULL);
            ImGui::InputScalar("Max Slew Rate (%/s)", ImGuiDataType_Double,
                               &motor_slew_rate, NULL);
        }
        if (ImGui::CollapsingHeader("Dynamic Constants"))
        {
            ImGui::InputScalar("Kf divisor", ImGuiDataType_Double, &Kf_divisor_,
                               NULL);
        }
        if (ImGui::CollapsingHeader("Environment"))
        {
            ImGui::InputScalar("Gravity (m/s^2)", ImGuiDataType_Double, &g,
                               NULL);
            ImGui::InputScalar("Disturbance Mean", ImGuiDataType_Double,
                               &dist_mean, NULL);
            ImGui::InputScalar("Disturbance Std. Dev", ImGuiDataType_Double,
                               &dist_stddev, NULL);
        }
        ImGui::EndTabItem();
    }

    // End the disabled part if gains should not be changed
    if (!is_enabled)
        ImGui::EndDisabled();

    return true;
}

Eigen::VectorXd QuadcopterDynamics::GetState() { return this->x; }

void QuadcopterDynamics::IdleLoop() { SetVehicleParams(); }

void QuadcopterDynamics::SetVehicleParams()
{
    // Force and moment constants. F = K_f w^2, M = K_b w^2
    // TODO: This is arbitrary, but sort of based on the motor specs
    // Max force is 1.522 kg for a 5.1x3.1x3 propeller. This assumes 16V when we
    // have 14.8V equipped. Let's take 1.522 * 14.8 / 16 = 1.40785 kg to be max
    // pull. (Force = 1.40785 * 9.81 = 13.811).

    // Max RPM (no-load) is 2750 KV * 14.8V = 40,700 RPM -> 4262 rad/s.
    // Assuming a load makes for 80% max speed -> 3409.6 rad/s

    // Thus, Kf = max_force / max_omega^2
    K_f = max_motor_force / (max_omega * max_omega);

    // Based on nothing but the feeling that torque of the props is much smaller
    // than the force of the props, we divide K_f by some factor to get K_m
    K_m = K_f / Kf_divisor_;

    // Save the parameters that are currently set.
    SaveParams();
}

void QuadcopterDynamics::LoadDefaultParams()
{
    // Set the mass
    m = 1.00; // kg

    // Set gravity
    g = 9.81;

    // Set the inertia
    // TODO: Make this configurable
    Ixx = 0.007; // kg/m^2
    Iyy = 0.006; // kg/m^2
    Izz = 0.009; // kg/m^2

    // Limit the motor's slew rate
    motor_slew_rate = 0.4 / 0.01; // Limit to changes of 40% every 0.01 sec

    // Force and moment constants. F = K_f w^2, M = K_b w^2
    // TODO: This is arbitrary, but sort of based on the motor specs
    // Max force is 1.522 kg for a 5.1x3.1x3 propeller. This assumes 16V when we
    // have 14.8V equipped. Let's take 1.522 * 14.8 / 16 = 1.40785 kg to be max
    // pull. (Force = 1.40785 * 9.81 = 13.811).
    max_motor_force = 13.811;

    // Max RPM (no-load) is 2750 KV * 14.8V = 40,700 RPM -> 4262 rad/s.
    // Assuming a load makes for 80% max speed -> 3409.6 rad/s
    max_omega = 3409.6;

    // Based on nothing but the feeling that torque of the props is much smaller
    // than the force of the props, we divide K_f by 20 to get K_m
    Kf_divisor_ = 20.0;

    // Simplify Ixy, Ixz, Iyz = 0 since they are all very small
    Ixy = 0;
    Ixz = 0;
    Iyz = 0;

    // Arm length
    dx_arm = 0.097;
    dy_arm = 0.15; // m

    // Disturbances
    dist_mean = 0.0;
    dist_stddev = 0.5;
}

void QuadcopterDynamics::SaveParams()
{
    std::string param_string =
        ""
        "// Set the mass\n"
        "m = " +
        std::to_string(m) +
        "; // kg\n"
        "\n"
        "// Set gravity\n"
        "g = " +
        std::to_string(g) +
        ";\n"
        "\n"
        "// Set the inertia\n"
        "Ixx = " +
        std::to_string(Ixx) +
        "; // kg/m^2\n"
        "Iyy = " +
        std::to_string(Iyy) +
        "; // kg/m^2\n"
        "Izz = " +
        std::to_string(Izz) +
        "; // kg/m^2\n"
        "\n"
        "// Limit the motor's slew rate\n"
        "motor_slew_rate = " +
        std::to_string(motor_slew_rate) +
        "; // Limit to changes of 40% every 0.01 sec\n"
        "\n"
        "// Force and moment constants. F = K_f w^2, M = K_b w^2\n"
        "// TODO: This is arbitrary, but sort of based on the motor specs\n"
        "// Max force is 1.522 kg for a 5.1x3.1x3 propeller. This assumes 16V "
        "when we\n"
        "// have 14.8V equipped. Let's take 1.522 * 14.8 / 16 = 1.40785 kg to "
        "be max\n"
        "// pull. (Force = 1.40785 * 9.81 = 13.811).\n"
        "max_motor_force = " +
        std::to_string(max_motor_force) +
        ";\n"
        "\n"
        "// Max RPM (no-load) is 2750 KV * 14.8V = 40,700 RPM -> 4262 rad/s.\n"
        "// Assuming a load makes for 80% max speed -> 3409.6 rad/s\n"
        "max_omega = " +
        std::to_string(max_omega) +
        ";\n"
        "\n"
        "// Based on nothing but the feeling that torque of the props is much "
        "smaller\n"
        "// than the force of the props, we divide K_f by 20 to get K_m\n"
        "Kf_divisor_ = " +
        std::to_string(Kf_divisor_) +
        ";\n"
        "\n"
        "// Simplify Ixy, Ixz, Iyz = 0 since they are all very small\n"
        "Ixy = " +
        std::to_string(Ixy) +
        ";\n"
        "Ixz = " +
        std::to_string(Ixz) +
        ";\n"
        "Iyz = " +
        std::to_string(Iyz) +
        ";\n"
        "\n"
        "// Arm length\n"
        "dx_arm = " +
        std::to_string(dx_arm) +
        ";\n"
        "dy_arm = " +
        std::to_string(dy_arm) +
        "; // m\n"
        "\n"
        "// Disturbances\n"
        "dist_mean = " +
        std::to_string(dist_mean) +
        ";\n"
        "dist_stddev = " +
        std::to_string(dist_stddev) + ";\n";

    // Save this string to the quadcopter parameters file (to load as default
    // via copy-paste later.) Gains export to TOML.
    // TODO: import from TOML
    quadcopter_param_file.open("quadcopter.toml",
                               std::ofstream::out | std::ofstream::trunc);
    quadcopter_param_file << param_string << std::flush;
    quadcopter_param_file.close();
}