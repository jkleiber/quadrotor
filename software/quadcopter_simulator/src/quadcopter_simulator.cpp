#include "quadcopter_simulator/quadcopter_simulator.h"

void QuadcopterSim::Init()
{
    // Set time to 0.
    clk_.Update(0.0);

    // Previous print at -100 to print initial conditions
    prev_print_time_ = -100.0;

    // Initial Conditions
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);

    // Initialize dynamics
    quadcopter_.Init(x0);

    // Zero out motor throttles.
    motor_throttles_ = Eigen::VectorXd::Zero(4);

    // Initialize logging.
    InitLogging();
}

void QuadcopterSim::InitLogging()
{
    // Initialize logging
    if (state_log_.IsActive())
    {
        state_log_.CloseLog();
    }

    // TODO: time based logging
    state_log_.Init("state_log.csv");
    state_log_.LogHeaders("u, v, w, p, q, r, x, y, z, roll, pitch, yaw");
}

void QuadcopterSim::Update()
{
    // Print periodically while the simulation runs
    if (clk_.GetTime() - prev_print_time_ > 1.0)
    {
        Eigen::VectorXd x = quadcopter_.GetState();
        prev_print_time_ = clk_.GetTime();
        std::cout << "t=" << prev_print_time_ << "\theight=" << x(8)
                  << "\troll=" << x(9) << " pitch=" << x(10) << " yaw=" << x(11)
                  << std::endl;

        std::cout << "fl motor: " << motor_throttles_(2) << std::endl;
    }

    // Quadcopter dynamics
    quadcopter_.UpdateDynamics(motor_throttles_);

    // Log the current sim state.
    UpdateSimState();
    LogState();

    // Increment time
    clk_.Increment();
}

void QuadcopterSim::ApplyMotorControl(const double &front_left,
                                      const double &front_right,
                                      const double &back_left,
                                      const double &back_right)
{
    // This is the mapping the quadcopter dynamics use for motors.
    motor_throttles_(0) = front_right;
    motor_throttles_(1) = back_left;
    motor_throttles_(2) = front_left;
    motor_throttles_(3) = back_right;
}

SimState QuadcopterSim::GetSimState() const { return sim_state_; }

void QuadcopterSim::UpdateSimState()
{
    sim_state_.altitude = quadcopter_.GetState()(8);
}

void QuadcopterSim::LogState()
{
    // Log the current state.
    state_log_.LogVectorXd(quadcopter_.GetState());
}
